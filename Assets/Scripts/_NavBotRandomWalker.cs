using Cysharp.Threading.Tasks;
using System.Threading;
using UnityEngine;

namespace BotSystem
{
    [AddComponentMenu("[BotSystem]: Nav Bot Random Walker ( For Testing )")]
    public sealed class _NavBotRandomWalker : MonoBehaviour
    {
        [SerializeField]
        private _NavBotMovement movement;

        private void Start()
        {
            MoveLoop(this.GetCancellationTokenOnDestroy()).Forget();
        }

        private async UniTask MoveLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                var path = await NavPathfinder.Instance.GetPathAsync(
                    transform.position,
                    NavGrid.Instance.GetRandomPoint().Point.position,
                    ct);

                await movement.MovePath(path, ct);
            }
        }
    }
}
