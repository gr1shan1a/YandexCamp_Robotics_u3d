using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(VirtualSensors))]
[RequireComponent(typeof(ROSBridge))]
public sealed class GripperIRAutoClose : MonoBehaviour
{
    public enum GripPreset
    {
        Cube,
        Ball,
        AutoFromVision
    }

    [Header("References")]
    public VirtualSensors sensors;
    public ROSBridge rosBridge;
    public GripperController gripper;
    public RealVision realVision;

    [Header("Detection")]
    [Tooltip("Ignore simulated raycasts and react only to a fresh /sensor/data stream.")]
    public bool requireFreshRealSensorStream = true;
    [Min(0f)]
    [Tooltip("The IR signal must remain active for this long before the claw closes.")]
    public float detectionConfirmSeconds = 0.05f;
    [Min(0f)]
    [Tooltip("The IR signal must remain clear for this long before another grab is allowed.")]
    public float rearmAfterClearSeconds = 0.75f;
    [Tooltip("Keep this off on the real robot so a captured object stays latched until ResetLatch is called.")]
    public bool rearmAutomatically;

    [Header("Actions")]
    public bool stopDriveBeforeClosing = true;
    public bool mirrorCloseInUnity = true;
    [Tooltip("Cube uses S4=60, ball uses S4=70. Auto remembers the last YOLO class.")]
    public GripPreset gripPreset = GripPreset.AutoFromVision;
    [Range(0, 1)]
    [Tooltip("Class used if IR fires before YOLO supplies a class. 0=ball, 1=cube.")]
    public int fallbackTargetClassId = 0;
    [Min(0.05f)]
    [Tooltip("Repeat the physical S4 command at this interval after IR detection.")]
    public float closeRepeatInterval = 0.15f;
    [Min(0.1f)]
    [Tooltip("Keep the robot stopped and repeat the physical S4 command for this long.")]
    public float closeRepeatSeconds = 1.2f;
    [Range(0f, 1f)] public float unityCubeJawClosure = 0.5f;
    public bool logStateChanges = true;

    [Header("Diagnostics (read only)")]
    [SerializeField] bool sensorStreamReady;
    [SerializeField] bool targetDetected;
    [SerializeField] bool grabLatched;
    [SerializeField] float activeSeconds;
    [SerializeField] float clearSeconds;
    [SerializeField] int closeCommandCount;
    [SerializeField] int latchedTargetClassId = -1;

    float activeSince = -1f;
    float clearSince = -1f;
    float repeatCloseUntil = -1f;
    float nextCloseCommandTime = -1f;
    int lastVisionClassId = -1;

    public bool GrabLatched => grabLatched;
    public int CloseCommandCount => closeCommandCount;

    void Awake()
    {
        if (sensors == null)
        {
            sensors = GetComponent<VirtualSensors>();
        }

        if (rosBridge == null)
        {
            rosBridge = GetComponent<ROSBridge>();
        }

        if (gripper == null)
        {
            gripper = GripperController.FindController(transform);
        }

        if (realVision == null)
        {
            realVision = GetComponent<RealVision>();
        }
    }

    void OnEnable()
    {
        activeSince = -1f;
        clearSince = -1f;
        activeSeconds = 0f;
        clearSeconds = 0f;
    }

    void Update()
    {
        if (realVision != null && realVision.IsTargetVisible)
        {
            lastVisionClassId = realVision.TargetClassId;
        }

        if (sensors == null)
        {
            sensorStreamReady = false;
            targetDetected = false;
            return;
        }

        sensorStreamReady =
            !requireFreshRealSensorStream ||
            (sensors.useRealSensors && sensors.RealSensorsFresh);
        targetDetected = sensorStreamReady && sensors.gripperIR == 1;
        RepeatCloseCommandIfNeeded();

        if (targetDetected)
        {
            clearSince = -1f;
            clearSeconds = 0f;

            if (activeSince < 0f)
            {
                activeSince = Time.unscaledTime;
                if (logStateChanges)
                {
                    Debug.Log("[GripperIRAutoClose] Gripper IR detected a target.", this);
                }
            }

            activeSeconds = Time.unscaledTime - activeSince;
            if (!grabLatched && activeSeconds >= detectionConfirmSeconds)
            {
                CloseFromIr();
            }
            return;
        }

        activeSince = -1f;
        activeSeconds = 0f;

        if (!grabLatched)
        {
            clearSince = -1f;
            clearSeconds = 0f;
            return;
        }

        if (clearSince < 0f)
        {
            clearSince = Time.unscaledTime;
        }

        clearSeconds = Time.unscaledTime - clearSince;
        if (rearmAutomatically && clearSeconds >= rearmAfterClearSeconds)
        {
            grabLatched = false;
            latchedTargetClassId = -1;
            repeatCloseUntil = -1f;
            nextCloseCommandTime = -1f;
            clearSince = -1f;
            clearSeconds = 0f;
            if (logStateChanges)
            {
                Debug.Log("[GripperIRAutoClose] IR cleared; automatic grab rearmed.", this);
            }
        }
    }

    public void CloseFromIr()
    {
        if (grabLatched)
        {
            return;
        }

        grabLatched = true;
        latchedTargetClassId = GetTargetClassId();
        repeatCloseUntil =
            Time.unscaledTime + Mathf.Max(0.1f, closeRepeatSeconds);
        nextCloseCommandTime = Time.unscaledTime;
        SendCloseCommand();

        if (mirrorCloseInUnity && gripper != null)
        {
            if (latchedTargetClassId == 1)
            {
                gripper.SetJawClosure01(unityCubeJawClosure, true);
            }
            else
            {
                gripper.CloseGripper();
            }
        }

        if (logStateChanges)
        {
            Debug.Log(
                $"[GripperIRAutoClose] STOP + close command sent " +
                $"(IR={sensors?.gripperIR}, class={latchedTargetClassId}, " +
                $"count={closeCommandCount}).",
                this
            );
        }
    }

    void RepeatCloseCommandIfNeeded()
    {
        if (!grabLatched ||
            latchedTargetClassId < 0 ||
            Time.unscaledTime > repeatCloseUntil ||
            Time.unscaledTime < nextCloseCommandTime)
        {
            return;
        }

        SendCloseCommand();
    }

    void SendCloseCommand()
    {
        if (rosBridge == null)
        {
            return;
        }

        if (stopDriveBeforeClosing)
        {
            rosBridge.PublishStop();
        }

        rosBridge.CloseGripperForTargetClass(latchedTargetClassId);
        closeCommandCount++;
        nextCloseCommandTime =
            Time.unscaledTime + Mathf.Max(0.05f, closeRepeatInterval);
    }

    int GetTargetClassId()
    {
        switch (gripPreset)
        {
            case GripPreset.Ball:
                return 0;
            case GripPreset.AutoFromVision:
                return lastVisionClassId >= 0
                    ? lastVisionClassId
                    : Mathf.Clamp(fallbackTargetClassId, 0, 1);
            default:
                return 1;
        }
    }

    public void ResetLatch()
    {
        grabLatched = false;
        latchedTargetClassId = -1;
        repeatCloseUntil = -1f;
        nextCloseCommandTime = -1f;
        activeSince = -1f;
        clearSince = -1f;
        activeSeconds = 0f;
        clearSeconds = 0f;
    }
}
