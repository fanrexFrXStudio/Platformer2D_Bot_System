using UnityEngine;

namespace BotSystem
{
    public sealed class RigidbodyInterpolate : MonoBehaviour
    {
        [SerializeField]
        private Transform visual;

        [SerializeField]
        private bool ownerOnly;

        private Vector3 currentPosition, lastPosition;
        private float offsetY;

        private void Awake()
        {
            if (visual == null)
            {
                return;
            }
            visual.SetParent(null);
            lastPosition = currentPosition = transform.position;
            offsetY = transform.position.y - visual.position.y;
        }

        private void OnDestroy()
        {
            if (visual != null)
            {
                Destroy(visual.gameObject);
            }
        }

        private void FixedUpdate()
        {
            lastPosition = currentPosition;
        }

        private void LateUpdate()
        {
            if (visual == null)
            {
                return;
            }

            currentPosition = transform.position;
            currentPosition.y -= offsetY;

            var time = (Time.time - Time.fixedTime) / Time.fixedDeltaTime;
            time = Mathf.Clamp01(time);

            visual.SetPositionAndRotation(Vector3.Lerp(lastPosition, currentPosition, time), transform.rotation);
        }
    }
}
