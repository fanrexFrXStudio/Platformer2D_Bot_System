using Cysharp.Threading.Tasks;
using System.Collections.Generic;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BotSystem
{
    public class NavPath
    {
        public List<Vector2> CorePoints;
        public List<HashSet<int>> CorridorNodes;
    }

    public struct PathRequest
    {
        public float2 Start, Goal;
        public UniTaskCompletionSource<NavPath> Tcs;
        public CancellationToken Ct;
    }

    [BurstCompile]
    public struct AStarJob : IJob
    {
        [ReadOnly]
        public NavGraphData Graph;

        [ReadOnly]
        public NativeBitArray Occupancy;

        public int StartIdx, GoalIdx;
        public NativeList<float2> Result;

        public void Execute()
        {
            if (StartIdx < 0 || GoalIdx < 0)
            {
                return;
            }

            var count = Graph.Positions.Length;
            var gScore = new NativeArray<float>(count, Allocator.Temp);
            var cameFrom = new NativeArray<int>(count, Allocator.Temp);
            var closed = new NativeBitArray(count, Allocator.Temp);
            var openSet = new NativePriorityQueue(count, Allocator.Temp);

            for (var i = 0; i < count; i++)
            {
                gScore[i] = float.MaxValue;
                cameFrom[i] = -1;
            }

            gScore[StartIdx] = 0f;

            var distance = math.distance(Graph.Positions[StartIdx], Graph.Positions[GoalIdx]);
            openSet.Push(new Node(StartIdx, distance));

            while (openSet.Count > 0)
            {
                var curr = openSet.Pop().Index;

                if (curr == GoalIdx)
                {
                    Reconstruct(cameFrom);
                    break;
                }

                if (closed.IsSet(curr))
                {
                    continue;
                }

                closed.Set(curr, true);

                var start = Graph.NeighbourStart[curr];
                var nCnt = Graph.NeighbourCount[curr];

                for (var i = 0; i < nCnt; i++)
                {
                    var nIdx = Graph.Neighbours[start + i];

                    if (closed.IsSet(nIdx) || (Occupancy.IsSet(nIdx) && nIdx != GoalIdx && nIdx != StartIdx))
                    {
                        continue;
                    }

                    var cost = Graph.Costs[start + i];
                    var targetG = gScore[curr] + cost;

                    if (targetG < gScore[nIdx])
                    {
                        cameFrom[nIdx] = curr;
                        gScore[nIdx] = targetG;

                        var distToGoal = math.distance(Graph.Positions[nIdx], Graph.Positions[GoalIdx]);
                        openSet.Push(new Node(nIdx, targetG + distToGoal));
                    }
                }
            }

            gScore.Dispose();
            cameFrom.Dispose();
            closed.Dispose();
            openSet.Dispose();
        }

        private void Reconstruct(NativeArray<int> cameFrom)
        {
            var path = new NativeList<int>(Allocator.Temp);
            var curr = GoalIdx;

            while (curr != -1)
            {
                path.Add(curr);
                curr = cameFrom[curr];
            }

            for (var i = path.Length - 1; i >= 0; i--)
            {
                Result.Add(Graph.Positions[path[i]]);
            }

            path.Dispose();
        }

        private struct Node
        {
            public int Index;
            public float Priority;
            public Node(int i, float p)
            {
                Index = i;
                Priority = p;
            }
        }

        private struct NativePriorityQueue
        {
            public readonly int Count => heap.Length;

            private NativeList<Node> heap;

            public NativePriorityQueue(int cap, Allocator alc)
            {
                heap = new NativeList<Node>(cap, alc);
            }

            public void Push(Node n)
            {
                heap.Add(n);
                var i = heap.Length - 1;

                while (i > 0)
                {
                    var p = (i - 1) / 2;

                    if (heap[p].Priority <= heap[i].Priority)
                    {
                        break;
                    }

                    (heap[p], heap[i]) = (heap[i], heap[p]);
                    i = p;
                }
            }
            public Node Pop()
            {
                var r = heap[0];
                heap[0] = heap[^1];
                heap.RemoveAt(heap.Length - 1);
                var i = 0;

                while (true)
                {
                    var l = i * 2 + 1;
                    var rIdx = i * 2 + 2;
                    var s = i;

                    if (l < heap.Length && heap[l].Priority < heap[s].Priority)
                    {
                        s = l;
                    }

                    if (rIdx < heap.Length && heap[rIdx].Priority < heap[s].Priority)
                    {
                        s = rIdx;
                    }

                    if (s == i)
                    {
                        break;
                    }

                    (heap[s], heap[i]) = (heap[i], heap[s]);
                    i = s;
                }

                return r;
            }
            public void Dispose()
            {
                heap.Dispose();
            }
        }
    }

    // const, todo
    [DefaultExecutionOrder(-100)]
    [AddComponentMenu("[BotSystem]: Nav Pathfinder")]
    [RequireComponent(typeof(NavGrid))]
    public sealed class NavPathfinder : MonoBehaviour
    {
        public static NavPathfinder Instance { get; private set; }

        [SerializeField]
        private int pathsPerFrame = 16;

        [SerializeField]
        private int pathWidth = 2;

        public int PathWidth => pathWidth;

        private NavGrid grid;
        private readonly Queue<PathRequest> queue = new();
        private readonly List<_NavBotMovement> bots = new();
        private JobHandle allJobs;

        private void Awake()
        {
            Instance = this;
            grid = GetComponent<NavGrid>();
        }

        private void OnDestroy()
        {
            allJobs.Complete();
        }

        public void RegisterBot(_NavBotMovement bot)
        {
            bots.Add(bot);
        }

        public void UnregisterBot(_NavBotMovement bot)
        {
            bots.Remove(bot);
        }

        public void CompleteAll()
        {
            allJobs.Complete();
        }

        public UniTask<NavPath> GetPathAsync(Vector2 start, Vector2 goal, CancellationToken ct = default)
        {
            var tcs = new UniTaskCompletionSource<NavPath>();

            queue.Enqueue(new PathRequest
            {
                Start = start,
                Goal = goal,
                Tcs = tcs,
                Ct = ct
            });

            return tcs.Task;
        }

        private void Update()
        {
            allJobs.Complete();
            allJobs = default;

            grid.OccupancyMask.Clear();
            foreach (var bot in bots)
            {
                var idx = bot.CurrentNodeIndex;
                if (idx != -1)
                {
                    grid.OccupancyMask.Set(idx, true);
                }
            }
        }

        private void LateUpdate()
        {
            var count = math.min(queue.Count, pathsPerFrame);
            if (count <= 0)
            {
                return;
            }

            var handles = new NativeArray<JobHandle>(count, Allocator.Temp);
            for (var i = 0; i < count; i++)
            {
                var req = queue.Dequeue();
                if (req.Ct.IsCancellationRequested)
                {
                    i--;
                    count--;
                    continue;
                }
                handles[i] = Schedule(req);
            }
            allJobs = JobHandle.CombineDependencies(handles);
            handles.Dispose();
        }

        private JobHandle Schedule(PathRequest req)
        {
            var res = new NativeList<float2>(Allocator.Persistent);
            var job = new AStarJob
            {
                Graph = grid.GraphData,
                Occupancy = grid.OccupancyMask,
                StartIdx = grid.FindNearestIndex(req.Start),
                GoalIdx = grid.FindNearestIndex(req.Goal),
                Result = res,
            };

            var h = job.Schedule();
            CompleteAsync(h, res, req).Forget();
            return h;
        }

        private async UniTaskVoid CompleteAsync(JobHandle h, NativeList<float2> res, PathRequest req)
        {
            await UniTask.WaitUntil(() => { return h.IsCompleted; });
            h.Complete();

            var navPath = (NavPath)null;

            if (res.Length > 0)
            {
                navPath = new NavPath
                {
                    CorePoints = new List<Vector2>(res.Length),
                    CorridorNodes = new List<HashSet<int>>(res.Length)
                };

                for (var i = 0; i < res.Length; i++)
                {
                    var pos = (Vector2)res[i];
                    navPath.CorePoints.Add(pos);

                    // BFS expansion for each point
                    var centerIdx = grid.FindNearestIndex(pos);
                    
                    if (centerIdx == -1)
                    {
                        navPath.CorridorNodes.Add(new HashSet<int>());
                        continue;
                    }

                    // If the node is on ground or in a house, we don't expand the corridor (width = 0)
                    var isGrounded = grid.IsGroundedAt(centerIdx);
                    var isInHouse = grid.IsInHouseAt(centerIdx);
                    var effectiveWidth = (isGrounded || isInHouse) ? 0 : pathWidth;

                    navPath.CorridorNodes.Add(GetCorridorNodes(centerIdx, effectiveWidth));
                }
            }

            res.Dispose();
            req.Tcs.TrySetResult(navPath);
        }

        private HashSet<int> GetCorridorNodes(int centerIdx, int width)
        {
            var result = new HashSet<int> { centerIdx };
            if (width <= 0)
            {
                return result;
            }

            var currentLayer = new List<int> { centerIdx };
            var graph = grid.GraphData;

            for (var w = 0; w < width; w++)
            {
                var nextLayer = new List<int>();
                foreach (var idx in currentLayer)
                {
                    var start = graph.NeighbourStart[idx];
                    var count = graph.NeighbourCount[idx];

                    for (var n = 0; n < count; n++)
                    {
                        var neighborIdx = graph.Neighbours[start + n];
                        if (result.Add(neighborIdx))
                        {
                            nextLayer.Add(neighborIdx);
                        }
                    }
                }
                currentLayer = nextLayer;
                if (currentLayer.Count == 0)
                {
                    break;
                }
            }

            return result;
        }
    }
}