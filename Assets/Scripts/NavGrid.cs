using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BotSystem
{
    [Serializable]
    public struct NavPointData
    {
        public Transform Point;
        public Transform[] Neighbours;
        public float[] NeighbourCosts;
        public bool IsGrounded;
        public bool IsInHouse;
    }

    public struct KDNode
    {
        public float2 Position;
        public int PointIndex;
        public int Left, Right, Axis;
    }

    public struct NavGraphData : IDisposable
    {
        public NativeArray<float2> Positions;
        public NativeArray<int> NeighbourStart, NeighbourCount, Neighbours;
        public NativeArray<float> Costs;
        public NativeArray<bool> IsGrounded;
        public NativeArray<bool> IsInHouse;

        public void Dispose()
        {
            if (Positions.IsCreated) { Positions.Dispose(); }
            if (NeighbourStart.IsCreated) { NeighbourStart.Dispose(); }
            if (NeighbourCount.IsCreated) { NeighbourCount.Dispose(); }
            if (Neighbours.IsCreated) { Neighbours.Dispose(); }
            if (Costs.IsCreated) { Costs.Dispose(); }
            if (IsGrounded.IsCreated) { IsGrounded.Dispose(); }
            if (IsInHouse.IsCreated) { IsInHouse.Dispose(); }
        }
    }

    /// <summary>
    /// Основной класс навигационной сетки. Управляет данными графа и запеканием.
    /// </summary>
    [AddComponentMenu("[BotSystem]: Nav Grid")]
    public sealed class NavGrid : MonoBehaviour
    {
        private const float GizmoNodeRadius = 0.1f; // Радиус отображения узла в редакторе
        // Expose ground layer mask for external use
        public LayerMask GroundMask => groundLayer;

        // Return world position of a cell by its index
        public Vector2 GetCellWorldPosition(int index)
        {
            if (bakedData == null || index < 0 || index >= bakedData.Length)
            {
                return Vector2.zero;
            }
            return bakedData[index].Point.position;
        }

        // Check if a cell is grounded
        public bool IsGroundedAt(int index)
        {
            if (!GraphData.IsGrounded.IsCreated || index < 0 || index >= GraphData.Positions.Length)
            {
                return false;
            }
            return GraphData.IsGrounded[index];
        }

        public bool IsInHouseAt(int index)
        {
            if (!GraphData.IsInHouse.IsCreated || index < 0 || index >= GraphData.Positions.Length)
            {
                return false;
            }
            return GraphData.IsInHouse[index];
        }

        [SerializeField]
        private float
            neighbourRadius = 3f,
            agentRadius = 0.5f;

        [SerializeField]
        private float
            groundRayCheckDistance = 1f,
            ceilingRayCheckDistance = 2f;

        [SerializeField]
        private LayerMask groundLayer;

        [SerializeField]
        private LayerMask houseLayer;

        [SerializeField]
        private float
            inHouseCheckRadius = 1f;

        [SerializeField]
        private float
            groundCost = 1f,
            airCost = 2.5f;
        
        [HideInInspector]
        [SerializeField]
        private NavPointData[] bakedData;

        public static NavGrid Instance
        {
            get; private set;
        }
        public NavGraphData GraphData
        {
            get; private set;
        }

        public NativeBitArray OccupancyMask;
        private NativeArray<KDNode> kdTree;

        private void Awake()
        {
            Instance = this;
            BuildNativeData();
        }

        private void OnDestroy()
        {
            if (NavPathfinder.Instance != null)
            {
                NavPathfinder.Instance.CompleteAll();
            }
            GraphData.Dispose();
            if (kdTree.IsCreated)
            {
                kdTree.Dispose();
            }
            if (OccupancyMask.IsCreated)
            {
                OccupancyMask.Dispose();
            }
        }

        private void BuildNativeData()
        {
            if (bakedData == null || bakedData.Length == 0)
            {
                return;
            }
            var count = bakedData.Length;
            var totalEdges = 0;
            foreach (var p in bakedData)
            {
                totalEdges += p.Neighbours?.Length ?? 0;
            }

            var pos = new NativeArray<float2>(count, Allocator.Persistent);
            var nStart = new NativeArray<int>(count, Allocator.Persistent);
            var nCount = new NativeArray<int>(count, Allocator.Persistent);
            var neighbours = new NativeArray<int>(totalEdges, Allocator.Persistent);
            var costs = new NativeArray<float>(totalEdges, Allocator.Persistent);
            var grounded = new NativeArray<bool>(count, Allocator.Persistent);
            var inHouse = new NativeArray<bool>(count, Allocator.Persistent);

            var lookup = new Dictionary<Transform, int>(count);
            for (var i = 0; i < count; i++)
            {
                lookup[bakedData[i].Point] = i;
            }

            var edgeIdx = 0;
            for (var i = 0; i < count; i++)
            {
                var p = bakedData[i];
                pos[i] = (Vector2)p.Point.position;
                nStart[i] = edgeIdx;
                nCount[i] = p.Neighbours?.Length ?? 0;
                grounded[i] = p.IsGrounded;
                inHouse[i] = p.IsInHouse;
                for (var j = 0; j < nCount[i]; j++)
                {
                    neighbours[edgeIdx] = lookup[p.Neighbours[j]];
                    costs[edgeIdx] = p.NeighbourCosts[j];
                    edgeIdx++;
                }
            }

            GraphData = new NavGraphData
            {
                Positions = pos,
                NeighbourStart = nStart,
                NeighbourCount = nCount,
                Neighbours = neighbours,
                Costs = costs,
                IsGrounded = grounded,
                IsInHouse = inHouse
            };
            OccupancyMask = new NativeBitArray(count, Allocator.Persistent);
            BuildKDTree(pos, count);
        }

        private void BuildKDTree(NativeArray<float2> positions, int count)
        {
            var indices = new int[count];
            for (var i = 0; i < count; i++)
            {
                indices[i] = i;
            }
            var nodes = new KDNode[count];
            var nodeCount = 0;
            BuildKDRecursive(positions, indices, 0, count, 0, nodes, ref nodeCount);
            kdTree = new NativeArray<KDNode>(nodes, Allocator.Persistent);
        }

        private int BuildKDRecursive(NativeArray<float2> positions, int[] indices, int start, int end, int axis, KDNode[] nodes, ref int nodeCount)
        {
            if (start >= end)
            {
                return -1;
            }
            Array.Sort(indices, start, end - start, Comparer<int>.Create((_, __) => positions[_][axis].CompareTo(positions[__][axis])));
            var mid = start + (end - start) / 2;
            var nodeIdx = nodeCount++;
            var pIdx = indices[mid];
            nodes[nodeIdx] = new KDNode
            {
                Position = positions[pIdx],
                PointIndex = pIdx,
                Axis = axis,
                Left = -1,
                Right = -1
            };
            nodes[nodeIdx].Left = BuildKDRecursive(positions, indices, start, mid, 1 - axis, nodes, ref nodeCount);
            nodes[nodeIdx].Right = BuildKDRecursive(positions, indices, mid + 1, end, 1 - axis, nodes, ref nodeCount);
            return nodeIdx;
        }

        public int FindNearestIndex(float2 position)
        {
            if (!kdTree.IsCreated)
            {
                return -1;
            }
            var bestIdx = -1;
            var bestSqr = float.MaxValue;
            SearchKD(0, position, ref bestIdx, ref bestSqr);
            return bestIdx;
        }

        private void SearchKD(int nodeIdx, float2 target, ref int bestIdx, ref float bestSqr)
        {
            if (nodeIdx == -1)
            {
                return;
            }
            var node = kdTree[nodeIdx];
            var sqr = math.distancesq(node.Position, target);
            if (sqr < bestSqr)
            {
                bestSqr = sqr;
                bestIdx = node.PointIndex;
            }
            var diff = target[node.Axis] - node.Position[node.Axis];
            SearchKD(diff < 0 ? node.Left : node.Right, target, ref bestIdx, ref bestSqr);
            if (diff * diff < bestSqr)
            {
                SearchKD(diff < 0 ? node.Right : node.Left, target, ref bestIdx, ref bestSqr);
            }
        }

        public NavPointData GetRandomPoint()
        {
            if (bakedData?.Length > 0)
            {
                return bakedData[UnityEngine.Random.Range(0, bakedData.Length)];
            }
            return default;
        }

        [ContextMenu("Bake")]
        private void Bake()
        {
            var childCount = transform.childCount;
            var valid = new List<Transform>();
            for (var i = 0; i < childCount; i++)
            {
                var t = transform.GetChild(i);
                if (t.gameObject.activeSelf && Physics2D.Raycast(t.position, Vector2.up, ceilingRayCheckDistance, groundLayer).collider == null)
                {
                    valid.Add(t);
                }
            }

            // 1. Сначала определяем параметры для каждой точки
            var isGrounded = new bool[valid.Count];
            var isInHouse = new bool[valid.Count];
            for (var i = 0; i < valid.Count; i++)
            {
                isGrounded[i] = Physics2D.Raycast(valid[i].position, Vector2.down, groundRayCheckDistance, groundLayer);
                isInHouse[i] = Physics2D.OverlapCircle(valid[i].position, inHouseCheckRadius, houseLayer) != null;
            }

            bakedData = new NavPointData[valid.Count];
            for (var i = 0; i < valid.Count; i++)
            {
                var posA = (Vector2)valid[i].position;
                var neighbours = new List<Transform>();
                var costs = new List<float>();
                
                for (var j = 0; j < valid.Count; j++)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    var v = valid[j];
                    var sqrDist = Vector2.SqrMagnitude((Vector2)v.position - posA);
                    if (sqrDist > neighbourRadius * neighbourRadius)
                    {
                        continue;
                    }

                    var dir = (Vector2)v.position - posA;
                    var dist = math.sqrt(sqrDist);
                    
                    var collisionMask = groundLayer;
                    if (isInHouse[i] || isInHouse[j])
                    {
                        collisionMask |= houseLayer;
                    }

                    if (Physics2D.CircleCast(posA, agentRadius, dir.normalized, dist, collisionMask).collider != null)
                    {
                        continue;
                    }

                    neighbours.Add(v);
                    
                    // Путь считается наземным ТОЛЬКО если обе точки на земле
                    var isGroundPath = isGrounded[i] && isGrounded[j];
                    costs.Add(dist * (isGroundPath ? groundCost : airCost));
                }

                bakedData[i] = new NavPointData
                {
                    Point = valid[i],
                    Neighbours = neighbours.ToArray(),
                    NeighbourCosts = costs.ToArray(),
                    IsGrounded = isGrounded[i],
                    IsInHouse = isInHouse[i]
                };
            }

            var totalEdges = 0;
            foreach (var p in bakedData)
            {
                totalEdges += p.Neighbours?.Length ?? 0;
            }
            Debug.Log($"Baked {bakedData.Length} points, {totalEdges} total edges.");
        }

        private void OnDrawGizmos()
        {
            if (bakedData == null)
            {
                return;
            }
            
            // Сначала найдем индексы заземленных точек для быстрой проверки в Gizmos
            var groundedLookup = new Dictionary<Transform, bool>();
            foreach (var p in bakedData)
            {
                groundedLookup[p.Point] = p.IsGrounded;
            }

            foreach (var p in bakedData)
            {
                if (p.Point == null)
                {
                    continue;
                }
                if (p.IsInHouse)
                {
                    Gizmos.color = Color.red;
                }
                else
                {
                    Gizmos.color = p.IsGrounded ? Color.cyan : Color.magenta;
                }
                Gizmos.DrawSphere(p.Point.position, GizmoNodeRadius);
                
                if (p.Neighbours == null)
                {
                    continue;
                }
                for (var i = 0; i < p.Neighbours.Length; i++)
                {
                    var neighbour = p.Neighbours[i];
                    if (neighbour == null)
                    {
                        continue;
                    }
                    
                    // Линия зеленая только если ОБЕ точки на земле
                    var targetIsGrounded = groundedLookup.ContainsKey(neighbour) && groundedLookup[neighbour];
                    Gizmos.color = (p.IsGrounded && targetIsGrounded) ? Color.green : Color.yellow;
                    
                    Gizmos.DrawLine(p.Point.position, neighbour.position);
                }
            }
        }
    }
}