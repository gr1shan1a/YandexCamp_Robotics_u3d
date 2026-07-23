using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

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

    [Header("Real ROS sensors")]
    public bool useRealSensors;
    public string realSensorTopic = "/sensor/data";
    public string realPwmTopic = "/sensor/pwm";
    [Min(0.1f)] public float realSensorTimeout = 0.5f;
    public bool logGripperIrChanges = true;

    [Header("Real sensor diagnostics (read only)")]
    [SerializeField] bool realSensorStreamFresh;
    [SerializeField] float realSensorAge = -1f;
    [SerializeField] int receivedGripperIR;
    [SerializeField] int receivedSensorPackets;

    public float Ultrasonic01 { get; private set; } = 1f;
    public float CentralIR { get; private set; }
    public float LeftIR { get; private set; }
    public float RightIR { get; private set; }
    public float GripperIR { get; private set; }
    public float realPwmLeft { get; private set; }
    public float realPwmRight { get; private set; }
    public bool RealSensorsFresh =>
        useRealSensors &&
        lastRealSensorTime >= 0f &&
        Time.unscaledTime - lastRealSensorTime <= realSensorTimeout;
    public float LastRealSensorAge =>
        lastRealSensorTime < 0f ? -1f : Time.unscaledTime - lastRealSensorTime;
    public int RealSensorPackets { get; private set; }

    public float ultrasonicDist => useRealSensors ? realUltrasonic01 : Ultrasonic01;
    public int leftIR => useRealSensors ? realLeftIR : Binary(LeftIR);
    public int rightIR => useRealSensors ? realRightIR : Binary(RightIR);
    public int gripperIR => useRealSensors ? realGripperIR : Binary(GripperIR);

    ROSConnection ros;
    float realUltrasonic01;
    int realLeftIR;
    int realRightIR;
    int realGripperIR;
    float lastRealSensorTime = -1f;
    bool subscriptionsRegistered;
    bool staleWarningLogged;

    void Start()
    {
        if (useRealSensors)
        {
            EnsureRealSensorSubscriptions();
        }
    }

    public void EnableRealSensors()
    {
        useRealSensors = true;
        EnsureRealSensorSubscriptions();
    }

    void EnsureRealSensorSubscriptions()
    {
        if (subscriptionsRegistered)
        {
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<QuaternionMsg>(realSensorTopic, ReceiveRealSensors);
        ros.Subscribe<Vector3Msg>(realPwmTopic, ReceiveRealPwm);
        subscriptionsRegistered = true;
        Debug.Log(
            $"[VirtualSensors] Subscribed to {realSensorTopic}; gripper IR is Quaternion.w.",
            this
        );
    }

    void Update()
    {
        Ultrasonic01 = ReadUltrasonic01();
        CentralIR = ReadBinaryIR(centralIRPoint, irRange, false);
        LeftIR = ReadBinaryIR(leftIRPoint, irRange, false);
        RightIR = ReadBinaryIR(rightIRPoint, irRange, false);
        GripperIR = ReadBinaryIR(gripperIRPoint, gripperRange, true);

        if (useRealSensors &&
            (lastRealSensorTime < 0f || Time.unscaledTime - lastRealSensorTime > realSensorTimeout))
        {
            // A missing sensor stream must look unsafe to the policy.
            realUltrasonic01 = 0f;
            realLeftIR = 1;
            realRightIR = 1;
            realGripperIR = 0;
        }

        realSensorStreamFresh = RealSensorsFresh;
        realSensorAge = LastRealSensorAge;
        receivedGripperIR = realGripperIR;
        receivedSensorPackets = RealSensorPackets;

        if (useRealSensors && !realSensorStreamFresh && !staleWarningLogged)
        {
            Debug.LogWarning(
                $"[VirtualSensors] {realSensorTopic} is STALE. " +
                "Autonomous gripper IR is unavailable until fresh packets arrive.",
                this
            );
            staleWarningLogged = true;
        }
        else if (realSensorStreamFresh && staleWarningLogged)
        {
            Debug.Log(
                $"[VirtualSensors] {realSensorTopic} restored; " +
                $"gripper IR={receivedGripperIR}.",
                this
            );
            staleWarningLogged = false;
        }
    }

    void ReceiveRealSensors(QuaternionMsg message)
    {
        int previousGripperIR = realGripperIR;
        float ultrasonicMeters = Mathf.Max(0f, (float)message.x);
        realUltrasonic01 = ultrasonicRange > 0f
            ? Mathf.Clamp01(ultrasonicMeters / ultrasonicRange)
            : 1f;
        realLeftIR = message.y > 0.5 ? 1 : 0;
        realRightIR = message.z > 0.5 ? 1 : 0;
        realGripperIR = message.w > 0.5 ? 1 : 0;
        lastRealSensorTime = Time.unscaledTime;
        RealSensorPackets++;

        if (logGripperIrChanges && previousGripperIR != realGripperIR)
        {
            Debug.Log(
                $"[VirtualSensors] GRIPPER IR={(realGripperIR == 1 ? "DETECTED" : "CLEAR")} " +
                $"(/sensor/data.w={message.w:F1}, packet={RealSensorPackets}).",
                this
            );
        }
    }

    void ReceiveRealPwm(Vector3Msg message)
    {
        realPwmLeft = (float)message.x;
        realPwmRight = (float)message.y;
    }

    static int Binary(float value)
    {
        return value > 0.5f ? 1 : 0;
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
