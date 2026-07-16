using UnityEngine;

public class VirtualSensors : MonoBehaviour
{
    [Header("Anchors")]
    public Transform centerPoint;
    public Transform centralIRPoint;
    public Transform leftIRPoint;
    public Transform rightIRPoint;
    public Transform gripperIRPoint;

    [Header("Ranges")]
    public float ultrasonicRange = 2.0f;
    public float irRange = 0.15f;
    public float gripperRange = 0.08f;
    public LayerMask obstacleMask = ~0;
    public string targetBallTag = "TargetBall";

    public float Ultrasonic01 { get; private set; } = 1f;
    public float CentralIR { get; private set; }
    public float LeftIR { get; private set; }
    public float RightIR { get; private set; }
    public float GripperIR { get; private set; }

    void Update()
    {
        Ultrasonic01 = ReadUltrasonic01();
        CentralIR = ReadBinaryIR(centralIRPoint, irRange, false);
        LeftIR = ReadBinaryIR(leftIRPoint, irRange, false);
        RightIR = ReadBinaryIR(rightIRPoint, irRange, false);
        GripperIR = ReadBinaryIR(gripperIRPoint, gripperRange, true);
    }

    float ReadUltrasonic01()
    {
        if (centerPoint == null)
        {
            return 1f;
        }

        float bestDistance = ultrasonicRange;
        float[] angles = { -15f, -7.5f, 0f, 7.5f, 15f };

        foreach (float angle in angles)
        {
            Vector3 direction = Quaternion.AngleAxis(angle, centerPoint.up) * centerPoint.forward;
            if (Physics.Raycast(centerPoint.position, direction, out RaycastHit hit, ultrasonicRange, obstacleMask))
            {
                if (!IsTargetBall(hit.collider))
                {
                    bestDistance = Mathf.Min(bestDistance, hit.distance);
                }
            }
        }

        return Mathf.Clamp01(bestDistance / ultrasonicRange);
    }

    float ReadBinaryIR(Transform point, float range, bool targetBallOnly)
    {
        if (point == null)
        {
            return 0f;
        }

        if (Physics.Raycast(point.position, point.forward, out RaycastHit hit, range, obstacleMask))
        {
            return targetBallOnly ? (IsTargetBall(hit.collider) ? 1f : 0f) : 1f;
        }

        return 0f;
    }

    bool IsTargetBall(Collider hit)
    {
        return hit != null && hit.gameObject.tag == targetBallTag;
    }

    void OnDrawGizmosSelected()
    {
        DrawRay(centerPoint, ultrasonicRange, Color.cyan);
        DrawRay(centralIRPoint, irRange, Color.magenta);
        DrawRay(leftIRPoint, irRange, Color.yellow);
        DrawRay(rightIRPoint, irRange, Color.yellow);
        DrawRay(gripperIRPoint, gripperRange, Color.green);
    }

    void DrawRay(Transform point, float range, Color color)
    {
        if (point == null)
        {
            return;
        }

        Gizmos.color = color;
        Gizmos.DrawLine(point.position, point.position + point.forward * range);
    }
}
