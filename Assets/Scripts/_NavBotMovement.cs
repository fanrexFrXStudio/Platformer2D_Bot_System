using Cysharp.Threading.Tasks;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace BotSystem
{
    /// <summary>
    /// Система перемещения бота на основе трансформов и ручной физики.
    /// </summary>
    [AddComponentMenu("New nav bot movement")]
    public sealed class _NavBotMovement : MonoBehaviour
    {
        // Внутренние константы для оптимизации и физики
        private const float GridUpdateThresholdSqr = 0.04f;    // Порог обновления индекса сетки (0.2м)
        private const float CurrentPosCheckThresholdSqr = 0.01f; // Порог проверки текущей позиции (0.1м)
        private const float GroundRaycastHeight = 0.5f;        // Высота, с которой пускаем луч вниз для прилипания
        private const float GroundRaycastDistance = 1.5f;      // Дистанция луча для поиска земли
        private const float GizmoNodeSize = 0.05f;             // Размер точек коридора в Гизмо

        [Header("Настройки скорости")]
        [SerializeField]
        private float moveSpeed = 5;      // Скорость ходьбы по горизонтали
        [SerializeField]
        private float flySpeed = 5;       // Скорость перемещения при полете
        [SerializeField]
        private float gravity = -10;      // Сила гравитации

        [Header("Настройки навигации")]
        [SerializeField]
        private float stopDistanceX = 0.5f; // Дистанция остановки по горизонтали
        [SerializeField]
        private float stopDistanceY = 0.5f; // Дистанция переключения на полет по вертикали

        [Header("Ручная физика (Ground/Ceiling)")]
        [SerializeField]
        private Vector2 groundCheckOffset = new(0, -0.5f); // Смещение точки проверки земли

        [SerializeField]
        private float groundCheckRadius = 0.2f;            // Радиус проверки земли

        [SerializeField]
        private Vector2 ceilingCheckOffset = new(0, 0.5f); // Смещение точки проверки потолка
        [SerializeField]
        private float ceilingCheckRadius = 0.2f;           // Радиус проверки потолка

        [SerializeField]
        private float ceilingHitVerticalVelocity = 0f;     // Скорость после удара головой о потолок

        [SerializeField]
        private float surfaceYOffset = 0.5f;               // Смещение бота от земли (высота ног)

        [Header("Визуализация (Gizmos)")]
        [SerializeField]
        private bool drawGizmo = true;

        [SerializeField]
        private float pointRadius = 0.3f; // Радиус основных точек пути

        [SerializeField]
        private Color
            pointToPointColor = Color.red,
            pointColor = Color.lightBlue,
            reachedPointToPointColor = Color.lightGreen,
            reachedPointColor = Color.green;

        [Header("Слои")]
        [SerializeField]
        private LayerMask ground; // Слой земли и зданий

        private float verticalVelocity;
        private NavPath currentPath;
        private int currentPointIndex;

        public int CurrentNodeIndex => GetCurrentGridIndex();

        private int cachedGridIndex = -1;
        private Vector2 lastCachedPos;

        /// <summary>
        /// Оптимизированное получение индекса текущего узла навигационной сетки.
        /// </summary>
        private int GetCurrentGridIndex()
        {
            if (NavGrid.Instance == null)
            {
                return -1;
            }

            var currentPos = (Vector2)transform.position;
            // Обновляем индекс только если бот прошел достаточное расстояние
            if (cachedGridIndex == -1 || Vector2.SqrMagnitude(currentPos - lastCachedPos) > GridUpdateThresholdSqr)
            {
                cachedGridIndex = NavGrid.Instance.FindNearestIndex(currentPos);
                lastCachedPos = currentPos;
            }
            return cachedGridIndex;
        }

        private void Start()
        {
            // Регистрация бота в навигаторе для системы обхода (Occupancy)
            if (NavPathfinder.Instance != null)
            {
                NavPathfinder.Instance.RegisterBot(this);
            }
        }

        private void OnDestroy()
        {
            if (NavPathfinder.Instance != null)
            {
                NavPathfinder.Instance.UnregisterBot(this);
            }
        }

        /// <summary>
        /// Основной метод прохождения пути.
        /// </summary>
        public async UniTask MovePath(NavPath path, CancellationToken ct = default)
        {
            try
            {
                currentPath = path;
                currentPointIndex = 0;

                while (currentPath != null && currentPointIndex < currentPath.CorePoints.Count && !ct.IsCancellationRequested)
                {
                    // Проверяем: не находимся ли мы уже в зоне текущей или ЛЮБОЙ будущей точки (срезание углов)
                    var myNode = GetCurrentGridIndex();
                    for (var i = currentPath.CorePoints.Count - 1; i >= currentPointIndex; i--)
                    {
                        if (currentPath.CorridorNodes[i].Contains(myNode))
                        {
                            currentPointIndex = i + 1;
                            break;
                        }
                    }

                    if (currentPointIndex >= currentPath.CorePoints.Count)
                    {
                        break;
                    }

                    await MoveToPoint(currentPath.CorePoints[currentPointIndex], ct);
                    currentPointIndex++;
                }
            }
            finally
            {
                currentPath = null;
                currentPointIndex = 0;
            }
        }

        /// <summary>
        /// Перемещение к конкретной точке пути с учетом физики.
        /// </summary>
        private async UniTask MoveToPoint(Vector2 pos, CancellationToken ct)
        {
            if (NavGrid.Instance == null)
            {
                return;
            }

            // Запоминаем тип целевой точки (земля или воздух) один раз
            var targetIsGridGrounded = NavGrid.Instance.IsGroundedAt(NavGrid.Instance.FindNearestIndex(pos));

            while (!ct.IsCancellationRequested)
            {
                var currentPos = (Vector2)transform.position;
                var diff = pos - currentPos;

                // Определяем: является ли текущий участок пути наземным
                var isGroundPath = IsGroundAt(currentPos) && targetIsGridGrounded;
                
                // Летим, если цель высоко и это не наземный склон
                var shouldFly = diff.y > stopDistanceY && !isGroundPath;

                // Проверка достижения через коридор (узлы сетки)
                var myNode = GetCurrentGridIndex();
                if (currentPath != null && currentPointIndex < currentPath.CorridorNodes.Count)
                {
                    if (currentPath.CorridorNodes[currentPointIndex].Contains(myNode))
                    {
                        return;
                    }
                }

                // Обычная проверка дистанции (на случай если сетка слишком грубая)
                if (Mathf.Abs(diff.x) <= stopDistanceX && Mathf.Abs(diff.y) <= stopDistanceY)
                {
                    return;
                }

                // Проверки столкновений через OverlapCircle
                var isGrounded = Physics2D.OverlapCircle(currentPos + groundCheckOffset, groundCheckRadius, ground);
                var isCeiling = Physics2D.OverlapCircle(currentPos + ceilingCheckOffset, ceilingCheckRadius, ground);

                Vector2 velocity;
                if (shouldFly)
                {
                    // Логика полета: движемся по вектору вверх/в бок
                    var moveDir = diff.x > stopDistanceX ? 1 : (diff.x < -stopDistanceX ? -1 : 0);
                    velocity = new Vector2(moveDir * moveSpeed, flySpeed);
                }
                else
                {
                    // Логика ходьбы
                    var moveDir = Mathf.Abs(diff.x) > stopDistanceX ? (diff.x > 0 ? 1 : -1) : 0;
                    
                    if (isGrounded)
                    {
                        // На земле скорость по Y следует за углом склона к цели
                        verticalVelocity = isGroundPath ? (diff.normalized * moveSpeed).y : 0;
                    }
                    else
                    {
                        // В воздухе (падение) применяем гравитацию
                        verticalVelocity += gravity * Time.fixedDeltaTime;
                    }
                    velocity = new Vector2(moveDir * moveSpeed, verticalVelocity);
                }

                // Обработка удара о потолок
                if (velocity.y > 0 && isCeiling)
                {
                    velocity.y = ceilingHitVerticalVelocity;
                }

                var nextPos = currentPos + velocity * Time.fixedDeltaTime;

                // Прилипание к земле (Snapping) для предотвращения тряски и "утопания"
                if (isGrounded && !shouldFly)
                {
                    var hit = Physics2D.Raycast(new(nextPos.x, nextPos.y + GroundRaycastHeight), Vector2.down, GroundRaycastDistance, ground);
                    if (hit.collider != null)
                    {
                        nextPos.y = hit.point.y + surfaceYOffset;
                        velocity.y = 0;
                    }
                }

                // Применение позиции напрямую к Трансформу (без Rigidbody)
                transform.position = new Vector3(nextPos.x, nextPos.y, 0);
                verticalVelocity = velocity.y;

                await UniTask.Yield(PlayerLoopTiming.FixedUpdate);
            }
        }

        /// <summary>
        /// Проверка: находится ли точка на земле согласно данным навигационной сетки.
        /// </summary>
        private bool IsGroundAt(Vector2 position)
        {
            if (NavGrid.Instance == null)
            {
                return false;
            }

            var currentPos = (Vector2)transform.position;
            // Если проверяем текущую позицию - используем кешированный индекс
            if (Vector2.SqrMagnitude(position - currentPos) < CurrentPosCheckThresholdSqr)
            {
                var idx = GetCurrentGridIndex();
                return idx != -1 && NavGrid.Instance.IsGroundedAt(idx);
            }

            var index = NavGrid.Instance.FindNearestIndex(position);
            return index != -1 && NavGrid.Instance.IsGroundedAt(index);
        }

        private void OnDrawGizmos()
        {
            if (drawGizmo)
            {
                var pos = (Vector2)transform.position;
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(pos + groundCheckOffset, groundCheckRadius);
                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(pos + ceilingCheckOffset, ceilingCheckRadius);
            }

            if (!drawGizmo || currentPath == null)
            {
                return;
            }

            var points = currentPath.CorePoints;
            for (var i = 0; i < points.Count; i++)
            {
                var reached = i < currentPointIndex;
                
                // Отрисовка основной точки пути
                Gizmos.color = reached ? reachedPointColor : pointColor;
                Gizmos.DrawSphere(points[i], pointRadius);

                // Отрисовка "облака" узлов коридора (Thick Path)
                var corridor = currentPath.CorridorNodes[i];
                var corridorColor = reached ? reachedPointColor : pointColor;
                corridorColor.a = 0.2f;
                Gizmos.color = corridorColor;

                foreach (var nodeIdx in corridor)
                {
                    var nodePos = NavGrid.Instance.GetCellWorldPosition(nodeIdx);
                    Gizmos.DrawSphere(nodePos, GizmoNodeSize);
                }

                // Линии между точками
                if (i < points.Count - 1)
                {
                    Gizmos.color = reached ? reachedPointToPointColor : pointToPointColor;
                    Gizmos.DrawLine(points[i], points[i + 1]);
                }
            }
        }
    }
}