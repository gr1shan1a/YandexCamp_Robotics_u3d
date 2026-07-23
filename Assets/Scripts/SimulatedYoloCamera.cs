using UnityEngine;

public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Scene references")]
    public Camera sourceCamera;
    public Transform targetBall;
    public Transform ignoreRoot;

    [Header("Detection")]
    public float maxDistance = 2.0f;
    public float horizontalFov = 40f;
    public LayerMask occlusionMask = ~0;

    public bool IsTargetVisible { get; private set; }
    public float TargetX { get; private set; }
    public float Distance01 { get; private set; } = 1f;
    public bool seesBall => IsTargetVisible;
    public float normalizedAngle => TargetX;
    public float normalizedDistance => Distance01;
    public float lastKnownBallDirection { get; private set; }
    public float maxViewDistance => maxDistance;

    void Awake()
    {
        if (sourceCamera == null)
        {
            sourceCamera = GetComponent<Camera>();
        }

        if (sourceCamera == null)
        {
            sourceCamera = GetComponentInChildren<Camera>();
        }

        if (sourceCamera == null)
        {
            sourceCamera = Camera.main;
        }

        if (ignoreRoot == null)
        {
            ignoreRoot = transform;
        }
    }

    void Update()
    {
        UpdateDetection();
    }

    public void UpdateDetection()
    {
        IsTargetVisible = false;
        TargetX = 0f;
        Distance01 = 1f;

        if (sourceCamera == null || targetBall == null)
        {
            return;
        }

        Transform cameraTransform = sourceCamera.transform;
        Vector3 toBall = targetBall.position - cameraTransform.position;
        float distance = toBall.magnitude;

        if (distance > maxDistance)
        {
            return;
        }

        Vector3 localBall = cameraTransform.InverseTransformPoint(targetBall.position);
        if (localBall.z <= 0f)
        {
            return;
        }

        float halfHorizontalFov = horizontalFov * 0.5f;
        float horizontalAngle = Mathf.Atan2(localBall.x, localBall.z) * Mathf.Rad2Deg;
        if (Mathf.Abs(horizontalAngle) > halfHorizontalFov)
        {
            return;
        }

        Vector3 viewport = sourceCamera.WorldToViewportPoint(targetBall.position);
        if (viewport.y < 0f || viewport.y > 1f)
        {
            return;
        }

        if (IsOccluded(cameraTransform.position, toBall.normalized, distance))
        {
            return;
        }

        IsTargetVisible = true;
        TargetX = Mathf.Clamp(viewport.x * 2f - 1f, -1f, 1f);
        Distance01 = Mathf.Clamp01(distance / maxDistance);
        lastKnownBallDirection = TargetX;
    }

    bool IsOccluded(Vector3 origin, Vector3 direction, float distance)
    {
        RaycastHit[] hits = Physics.RaycastAll(origin, direction, distance, occlusionMask);
        System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

        foreach (RaycastHit hit in hits)
        {
            Transform hitTransform = hit.transform;
            if (hitTransform == targetBall || hitTransform.IsChildOf(targetBall))
            {
                return false;
            }

            if (ignoreRoot != null && (hitTransform == ignoreRoot || hitTransform.IsChildOf(ignoreRoot)))
            {
                continue;
            }

            return true;
        }

        return false;
    }
}

// Legacy camera API retained for RobotBrain models trained before SimulatedYoloCamera.
// New scenes should assign Simulated Yolo and leave Virtual Camera empty.
public class VirtualCamera : MonoBehaviour
{
    public Camera sourceCamera;
    public Transform targetBall;
    [Min(0.1f)] public float maxViewDistance = 6f;
    [Range(1f, 179f)] public float horizontalFov = 60f;
    public LayerMask occlusionMask = ~0;

    public bool seesBall { get; private set; }
    public float normalizedAngle { get; private set; }
    public float normalizedDistance { get; private set; } = 1f;
    public float lastKnownBallDirection { get; private set; }

    void Awake()
    {
        if (sourceCamera == null)
        {
            sourceCamera = GetComponent<Camera>();
        }

        if (sourceCamera == null)
        {
            sourceCamera = GetComponentInChildren<Camera>();
        }
    }

    void Update()
    {
        seesBall = false;
        normalizedAngle = 0f;
        normalizedDistance = 1f;

        if (sourceCamera == null || targetBall == null)
        {
            return;
        }

        Vector3 localTarget = sourceCamera.transform.InverseTransformPoint(targetBall.position);
        if (localTarget.z <= 0f)
        {
            return;
        }

        float angle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
        float distance = localTarget.magnitude;
        if (Mathf.Abs(angle) > horizontalFov * 0.5f || distance > maxViewDistance)
        {
            return;
        }

        Vector3 direction = (targetBall.position - sourceCamera.transform.position).normalized;
        if (Physics.Raycast(
                sourceCamera.transform.position,
                direction,
                out RaycastHit hit,
                distance,
                occlusionMask,
                QueryTriggerInteraction.Ignore) &&
            hit.transform != targetBall &&
            !hit.transform.IsChildOf(targetBall))
        {
            return;
        }

        seesBall = true;
        normalizedAngle = Mathf.Clamp(angle / (horizontalFov * 0.5f), -1f, 1f);
        normalizedDistance = Mathf.Clamp01(distance / maxViewDistance);
        lastKnownBallDirection = normalizedAngle;
    }
}
