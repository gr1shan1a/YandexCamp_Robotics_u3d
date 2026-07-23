using System;
using System.Text;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Parameterized keyboard control for the GFS-X arm, claw and pan/tilt camera.
/// The Unity twin always works. Direct physical joint control requires a
/// /cmd_servo subscriber that accepts geometry_msgs/Vector3:
/// x = servo channel, y = absolute angle in degrees.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(ROSBridge))]
public class ROSArmKeyboardTeleop : MonoBehaviour
{
    [Serializable]
    public sealed class ServoJoint
    {
        public string label = "Joint";
        [Tooltip("Turns this physical servo channel on or off.")]
        public bool enabled = true;
        [Tooltip("PCA9685/XiaoR servo channel on the physical robot.")]
        [Range(1, 8)] public int channel = 1;
        [Tooltip("First safe mechanical limit. For the claw this is the open angle.")]
        [Range(15f, 160f)] public float minAngle = 60f;
        [Tooltip("Second safe mechanical limit. For the claw this is the maximum allowed closure.")]
        [Range(15f, 160f)] public float maxAngle = 120f;
        [Tooltip("Angle used when Play starts and when 1/R resets the controls.")]
        [Range(15f, 160f)] public float startAngle = 90f;
        [Tooltip("Movement speed while the corresponding key is held.")]
        [Range(1f, 60f)] public float degreesPerSecond = 12f;
        [Tooltip("Reverse this joint without changing the keyboard mapping.")]
        public bool invertInput;

        [Tooltip("Live commanded angle. Play always begins from Start Angle.")]
        [Range(15f, 160f)] public float currentAngle = 90f;
        [NonSerialized] public float targetAngle;
        [NonSerialized] public bool dirty;
        bool initialized;

        public float LowAngle => Mathf.Min(minAngle, maxAngle);
        public float HighAngle => Mathf.Max(minAngle, maxAngle);
        public float DisplayCurrentAngle => initialized ? currentAngle : Clamp(startAngle);
        public float DisplayTargetAngle => initialized ? targetAngle : Clamp(startAngle);

        public void Initialize()
        {
            currentAngle = Clamp(startAngle);
            targetAngle = currentAngle;
            dirty = true;
            initialized = true;
        }

        public bool Move(float input, float deltaTime)
        {
            EnsureInitialized();
            if (!enabled || Mathf.Approximately(input, 0f))
            {
                return false;
            }

            float direction = invertInput ? -input : input;
            float next = Clamp(
                currentAngle + Mathf.Clamp(direction, -1f, 1f) * degreesPerSecond * deltaTime
            );
            if (Mathf.Abs(next - currentAngle) < 0.0001f)
            {
                return false;
            }

            currentAngle = next;
            targetAngle = currentAngle;
            dirty = true;
            return true;
        }

        public void SetNormalized(float normalized)
        {
            EnsureInitialized();
            float low = Mathf.Min(minAngle, maxAngle);
            float high = Mathf.Max(minAngle, maxAngle);
            currentAngle = Mathf.Lerp(low, high, Mathf.Clamp01(normalized));
            targetAngle = currentAngle;
            dirty = true;
        }

        public void SetTargetAngle(float angle)
        {
            EnsureInitialized();
            targetAngle = Clamp(angle);
        }

        public void SetCurrentAngle(float angle)
        {
            EnsureInitialized();
            currentAngle = Clamp(angle);
            targetAngle = currentAngle;
            dirty = true;
        }

        public bool UpdateTowardsTarget(float deltaTime)
        {
            EnsureInitialized();
            if (!enabled)
            {
                return true;
            }

            float next = Mathf.MoveTowards(
                currentAngle,
                targetAngle,
                degreesPerSecond * Mathf.Max(0f, deltaTime)
            );
            if (Mathf.Abs(next - currentAngle) >= 0.0001f)
            {
                currentAngle = next;
                dirty = true;
            }

            return Mathf.Abs(currentAngle - targetAngle) < 0.01f;
        }

        public bool MoveTowardsNormalized(float normalized, float deltaTime)
        {
            EnsureInitialized();
            if (!enabled)
            {
                return true;
            }

            float low = Mathf.Min(minAngle, maxAngle);
            float high = Mathf.Max(minAngle, maxAngle);
            targetAngle = Mathf.Lerp(low, high, Mathf.Clamp01(normalized));
            return UpdateTowardsTarget(deltaTime);
        }

        public void ResetTarget()
        {
            EnsureInitialized();
            targetAngle = Clamp(startAngle);
        }

        public void ValidateSettings()
        {
            channel = Mathf.Clamp(channel, 1, 8);
            minAngle = Mathf.Clamp(minAngle, 15f, 160f);
            maxAngle = Mathf.Clamp(maxAngle, 15f, 160f);
            if (minAngle > maxAngle)
            {
                float oldMin = minAngle;
                minAngle = maxAngle;
                maxAngle = oldMin;
            }

            startAngle = Mathf.Clamp(startAngle, minAngle, maxAngle);
            degreesPerSecond = Mathf.Clamp(degreesPerSecond, 1f, 60f);

            if (initialized)
            {
                currentAngle = Clamp(currentAngle);
                targetAngle = Clamp(targetAngle);
            }
        }

        float Clamp(float angle)
        {
            float low = Mathf.Max(15f, Mathf.Min(minAngle, maxAngle));
            float high = Mathf.Min(160f, Mathf.Max(minAngle, maxAngle));
            return Mathf.Clamp(angle, low, high);
        }

        void EnsureInitialized()
        {
            if (!initialized)
            {
                Initialize();
            }
        }
    }

    [Header("ROS")]
    public ROSBridge bridge;
    public bool enableTeleoperation = true;
    [Tooltip("Enable after the rover-side /cmd_servo subscriber has been started.")]
    public bool publishDirectServoCommands = true;
    [Min(1f)] public float directServoPublishRateHz = 20f;
    public bool logCommands = true;

    [Header("Servo diagnostics")]
    [Tooltip("Print one complete servo table when Play starts.")]
    public bool logServoSnapshotOnStart = true;
    [Tooltip("Print a compact block when commanded servo angles change.")]
    public bool logServoChanges = true;
    [Tooltip("Minimum delay between grouped servo change messages.")]
    [Range(0.05f, 2f)] public float servoLogIntervalSeconds = 0.25f;
    [Tooltip("Minimum accumulated angle change required for a new log line.")]
    [Range(0.01f, 5f)] public float servoLogThresholdDegrees = 0.1f;

    [Header("Legacy Pi macros")]
    [Tooltip("Uses current /cmd_gripper commands while direct servo control is disabled.")]
    public bool useLegacyMacrosWhenDirectDisabled;
    [Tooltip("DANGEROUS: the rover's old command 1 jumps S2 from 160 to 20 degrees. Keep this off.")]
    public bool allowLegacyWholeArmMacros;
    public int prepareToGrabCommand = 1;
    public int closeClawCommand = 2;
    public int resetArmCommand = 3;
    public int openClawCommand = 4;

    [Header("Physical arm servos")]
    public ServoJoint armRotationServo = new ServoJoint
    {
        label = "Main arm (2/3, S1)",
        channel = 1,
        minAngle = 70f,
        maxAngle = 110f,
        startAngle = 90f,
        degreesPerSecond = 8f
    };
    public ServoJoint shoulderServo = new ServoJoint
    {
        label = "Elbow (F/G, S2)",
        channel = 2,
        minAngle = 120f,
        maxAngle = 160f,
        startAngle = 160f,
        degreesPerSecond = 8f
    };
    public ServoJoint elbowServo = new ServoJoint
    {
        label = "Claw rotation (Q/E, S3)",
        channel = 3,
        minAngle = 90f,
        maxAngle = 120f,
        startAngle = 90f,
        degreesPerSecond = 8f
    };
    public ServoJoint clawServo = new ServoJoint
    {
        label = "Claw grip presets (S4)",
        channel = 4,
        minAngle = 50f,
        maxAngle = 70f,
        startAngle = 50f,
        degreesPerSecond = 8f
    };

    [Header("Calibrated claw presets")]
    [Range(15f, 160f)] public float clawOpenAngle = 50f;
    [Range(15f, 160f)] public float clawCubeAngle = 60f;
    [Range(15f, 160f)] public float clawBallAngle = 70f;
    [Range(0f, 1f)] public float unityCubeJawClosure = 0.5f;

    [Header("Physical safety")]
    [Tooltip("The installed rover driver clamps all servo commands to 160 degrees.")]
    public bool enforceSafeElbowLimit = true;
    [Range(15f, 160f)] public float minimumSafeElbowAngle = 120f;
    [SerializeField, HideInInspector] int servoCalibrationVersion;
    const int CurrentServoCalibrationVersion = 2;

    [Header("Unity digital twin")]
    public GripperController gripper;
    public bool mirrorToUnity = true;
    public bool disableArmKeyboardTester = true;
    [Tooltip("Reverse only the simulated shoulder motion for keys 2/3.")]
    public bool invertUnityShoulder;
    [Tooltip("Reverse only the simulated elbow motion for keys 2/3 and F/G.")]
    public bool invertUnityElbow;
    [Tooltip("Reverse only the simulated gripper rotation for Q/E.")]
    public bool invertUnityWrist;
    [Tooltip("Reverse only the simulated claw opening direction for Z/C.")]
    public bool invertUnityClaw;
    [Range(1f, 60f)] public float unityJawDegreesPerSecond = 12f;
    [Range(0.1f, 1f)] public float unityJawClosurePerSecond = 0.7f;
    [Tooltip("0.75 prevents the simulated jaws from closing completely.")]
    [Range(0.1f, 1f)] public float unityMaxJawClosure = 0.75f;
    [Range(0f, 90f)] public float unityWristMinAngle;
    [Range(0f, 90f)] public float unityWristMaxAngle = 90f;
    [Range(1f, 90f)] public float unityWristDegreesPerSecond = 30f;

    [Header("Camera preview and ROS topics")]
    public Transform cameraPivot;
    public bool enableCameraControl = true;
    public bool publishCameraCommands = true;
    [Tooltip("Off uses /cmd_camera_pan and /cmd_camera_tilt. On uses /cmd_servo channels below.")]
    public bool useDirectCameraServos = true;
    [Tooltip("Also sends I/K through the rover's existing /cmd_camera_pan callback. Keep enabled until the rover accepts S7/S8 on /cmd_servo.")]
    public bool publishLegacyVerticalCameraFallback = true;
    [Min(1f)] public float cameraDegreesPerSecond = 20f;
    [Range(0f, 90f)] public float cameraPanMaxAngle = 20f;
    [Range(0f, 90f)] public float cameraTiltMaxAngle = 12f;
    public bool invertCameraPan;
    public bool invertCameraTilt;
    [Min(1f)] public float cameraPublishRateHz = 20f;

    [Header("Physical camera servos")]
    public ServoJoint cameraTiltServo = new ServoJoint
    {
        label = "Camera vertical (I/K, confirmed S7)",
        channel = 7,
        minAngle = 70f,
        maxAngle = 110f,
        startAngle = 90f,
        degreesPerSecond = 15f,
        invertInput = true
    };
    [Tooltip("The rover's vendor xr_socket.py initializes the second camera servo on S8. Its requested 0 degrees is clamped by the driver to 15 degrees.")]
    public ServoJoint cameraPanServo = new ServoJoint
    {
        label = "Camera horizontal (J/L, S8)",
        enabled = true,
        channel = 8,
        minAngle = 15f,
        maxAngle = 60f,
        startAngle = 20f,
        degreesPerSecond = 15f
    };

    float nextDirectServoPublishTime;
    float nextCameraPublishTime;
    float cameraPanAngle;
    float cameraTiltAngle;
    float jawClosure01;
    float wristAngle;
    Quaternion cameraBaseRotation;
    bool cameraPoseCaptured;
    bool armWasMoving;
    bool elbowWasMoving;
    bool warnedMissingKeyboard;
    bool warnedDirectServoDisabled;
    readonly float[] lastLoggedServoAngles = new float[6];
    bool servoLogCacheInitialized;
    float nextServoLogTime;

    struct KeyState
    {
        public bool keyboardAvailable;
        public bool resetPressed;
        public bool lowerHeld;
        public bool lowerPressed;
        public bool raiseHeld;
        public bool raisePressed;
        public bool bendElbowHeld;
        public bool straightenElbowHeld;
        public bool rotateLeftHeld;
        public bool rotateRightHeld;
        public bool closeHeld;
        public bool closePressed;
        public bool openHeld;
        public bool openPressed;
        public bool cubeGripPressed;
        public bool grabPressed;
        public bool releasePressed;
        public bool cameraLeftHeld;
        public bool cameraRightHeld;
        public bool cameraUpHeld;
        public bool cameraDownHeld;
        public bool logSnapshotPressed;
    }

    void Awake()
    {
        if (bridge == null)
        {
            bridge = GetComponent<ROSBridge>();
        }

        GripperController primary = GripperController.FindController(transform.root);
        if (primary != null)
        {
            gripper = primary;
        }

        if (disableArmKeyboardTester)
        {
            ArmKeyboardTester[] testers = GetComponentsInChildren<ArmKeyboardTester>(true);
            foreach (ArmKeyboardTester tester in testers)
            {
                tester.enabled = false;
            }
        }

        if (mirrorToUnity && gripper != null)
        {
            gripper.enabled = true;
            gripper.jawDegreesPerSecond = unityJawDegreesPerSecond;
            jawClosure01 = Mathf.Min(gripper.JawClosure01, unityMaxJawClosure);
        }

        if (cameraPivot == null)
        {
            Camera childCamera = GetComponentInChildren<Camera>(true);
            if (childCamera != null)
            {
                cameraPivot = childCamera.transform;
            }
        }

        if (cameraPivot != null)
        {
            cameraBaseRotation = cameraPivot.localRotation;
            cameraPoseCaptured = true;
        }

        InitializeServoSettings();
        ConfigureBridgePublishers();
    }

    void Start()
    {
        if (logCommands)
        {
            Debug.Log(
                "GFS-X arm keys: 2/3 main arm (S1), F/G elbow (S2), " +
                "Q/E claw rotation (S3), C/X open 50, V cube 60, " +
                "Z/Space ball 70, I/K camera vertical, J/L camera horizontal, " +
                "1/R reset, P log all servo angles.",
                this
            );
        }

        if (logServoSnapshotOnStart)
        {
            LogServoSnapshot("Play started");
        }
        PrimeServoLogCache();
    }

    void Update()
    {
        if (!enableTeleoperation)
        {
            return;
        }

        KeyState keys = ReadKeys();
        if (!keys.keyboardAvailable)
        {
            if (!warnedMissingKeyboard)
            {
                Debug.LogError("ROSArmKeyboardTeleop: Unity did not detect a keyboard.", this);
                warnedMissingKeyboard = true;
            }
            return;
        }

        if (keys.logSnapshotPressed)
        {
            LogServoSnapshot("P key");
        }

        if (keys.resetPressed)
        {
            ResetAll(true);
            return;
        }

        float wholeArmInput = (keys.raiseHeld ? 1f : 0f) - (keys.lowerHeld ? 1f : 0f);
        float elbowFineInput = (keys.straightenElbowHeld ? 1f : 0f) -
                               (keys.bendElbowHeld ? 1f : 0f);
        float rotationInput = (keys.rotateRightHeld ? 1f : 0f) -
                              (keys.rotateLeftHeld ? 1f : 0f);
        float clawInput = 0f;

        WarnIfDirectServoControlIsRequired(
            wholeArmInput,
            elbowFineInput,
            rotationInput,
            clawInput,
            keys
        );

        UpdateUnityArm(wholeArmInput, elbowFineInput, rotationInput, clawInput, keys);
        UpdatePhysicalArm(wholeArmInput, elbowFineInput, rotationInput, clawInput, keys);
        UpdateCamera(keys);
        PublishDirtyServos();
        UpdateLegacyMacros(keys);
        LogChangedServoAngles();
    }

    void UpdateUnityArm(
        float wholeArmInput,
        float elbowFineInput,
        float rotationInput,
        float clawInput,
        KeyState keys
    )
    {
        if (!mirrorToUnity || gripper == null)
        {
            return;
        }

        if (!Mathf.Approximately(wholeArmInput, 0f))
        {
            float unityShoulderInput = invertUnityShoulder ? -wholeArmInput : wholeArmInput;
            gripper.JogArm(unityShoulderInput, Time.deltaTime);
            armWasMoving = true;
        }
        else if (armWasMoving)
        {
            gripper.StopArm();
            armWasMoving = false;
        }

        float combinedElbow = elbowFineInput;
        if (invertUnityElbow)
        {
            combinedElbow = -combinedElbow;
        }
        if (!Mathf.Approximately(combinedElbow, 0f))
        {
            gripper.JogElbow(combinedElbow, Time.deltaTime);
            elbowWasMoving = true;
        }
        else if (elbowWasMoving)
        {
            gripper.StopElbow();
            elbowWasMoving = false;
        }

        if (!Mathf.Approximately(rotationInput, 0f))
        {
            float unityRotationInput = invertUnityWrist ? -rotationInput : rotationInput;
            wristAngle = Mathf.Clamp(
                wristAngle + unityRotationInput * unityWristDegreesPerSecond * Time.deltaTime,
                Mathf.Min(unityWristMinAngle, unityWristMaxAngle),
                Mathf.Max(unityWristMinAngle, unityWristMaxAngle)
            );
            gripper.SetWristAngleDegrees(wristAngle);
        }

        if (keys.cubeGripPressed)
        {
            jawClosure01 = Mathf.Clamp01(unityCubeJawClosure);
            gripper.SetJawClosure01(jawClosure01);
        }

        if (keys.closePressed || keys.grabPressed)
        {
            jawClosure01 = unityMaxJawClosure;
            gripper.SetJawClosure01(jawClosure01, true);
        }
        else if (keys.openPressed || keys.releasePressed)
        {
            jawClosure01 = 0f;
            gripper.SetJawClosure01(0f);
        }
    }

    void UpdatePhysicalArm(
        float wholeArmInput,
        float elbowFineInput,
        float rotationInput,
        float clawInput,
        KeyState keys
    )
    {
        if (!publishDirectServoCommands)
        {
            return;
        }

        armRotationServo.Move(wholeArmInput, Time.unscaledDeltaTime);
        shoulderServo.Move(elbowFineInput, Time.unscaledDeltaTime);
        elbowServo.Move(rotationInput, Time.unscaledDeltaTime);

        if (keys.closePressed || keys.grabPressed)
        {
            clawServo.SetTargetAngle(clawBallAngle);
        }
        else if (keys.cubeGripPressed)
        {
            clawServo.SetTargetAngle(clawCubeAngle);
        }
        else if (keys.openPressed || keys.releasePressed)
        {
            clawServo.SetTargetAngle(clawOpenAngle);
        }

        armRotationServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
        shoulderServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
        elbowServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
        clawServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
    }

    void UpdateCamera(KeyState keys)
    {
        if (!enableCameraControl)
        {
            return;
        }

        float panInput = (keys.cameraRightHeld ? 1f : 0f) -
                         (keys.cameraLeftHeld ? 1f : 0f);
        float tiltInput = (keys.cameraUpHeld ? 1f : 0f) -
                          (keys.cameraDownHeld ? 1f : 0f);

        if (invertCameraPan)
        {
            panInput = -panInput;
        }
        if (invertCameraTilt)
        {
            tiltInput = -tiltInput;
        }

        cameraPanAngle = Mathf.Clamp(
            cameraPanAngle + panInput * cameraDegreesPerSecond * Time.unscaledDeltaTime,
            -cameraPanMaxAngle,
            cameraPanMaxAngle
        );
        cameraTiltAngle = Mathf.Clamp(
            cameraTiltAngle + tiltInput * cameraDegreesPerSecond * Time.unscaledDeltaTime,
            -cameraTiltMaxAngle,
            cameraTiltMaxAngle
        );

        if (cameraPoseCaptured)
        {
            cameraPivot.localRotation = cameraBaseRotation * Quaternion.Euler(
                -cameraTiltAngle,
                cameraPanAngle,
                0f
            );
        }

        if (useDirectCameraServos && publishDirectServoCommands)
        {
            cameraPanServo.Move(panInput, Time.unscaledDeltaTime);
            cameraTiltServo.Move(tiltInput, Time.unscaledDeltaTime);
            cameraPanServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
            cameraTiltServo.UpdateTowardsTarget(Time.unscaledDeltaTime);
        }

        bool sendLegacyCameraTopics = !useDirectCameraServos ||
                                      publishLegacyVerticalCameraFallback;
        if (publishCameraCommands && sendLegacyCameraTopics && bridge != null &&
            Time.unscaledTime >= nextCameraPublishTime)
        {
            // The current rover master has /cmd_camera_pan on S7 only. With
            // swapCameraTopics enabled, PublishCameraTilt reaches that callback.
            bridge.PublishCameraTilt(cameraTiltAngle);

            // Horizontal fallback is useful only after /cmd_camera_tilt exists.
            if (!useDirectCameraServos)
            {
                bridge.PublishCameraPan(cameraPanAngle);
            }

            nextCameraPublishTime = Time.unscaledTime + 1f / cameraPublishRateHz;
        }
    }

    void PublishDirtyServos()
    {
        if (!publishDirectServoCommands || bridge == null ||
            Time.unscaledTime < nextDirectServoPublishTime)
        {
            return;
        }

        PublishJoint(armRotationServo);
        PublishJoint(shoulderServo);
        PublishJoint(elbowServo);
        PublishJoint(clawServo);

        if (useDirectCameraServos)
        {
            PublishJoint(cameraPanServo);
            PublishJoint(cameraTiltServo);
        }

        nextDirectServoPublishTime = Time.unscaledTime + 1f / directServoPublishRateHz;
    }

    void PublishJoint(ServoJoint joint)
    {
        if (joint == null || !joint.enabled || !joint.dirty)
        {
            return;
        }

        bridge.PublishServoAngle(joint.channel, joint.currentAngle);
        joint.dirty = false;
    }

    void UpdateLegacyMacros(KeyState keys)
    {
        if (publishDirectServoCommands || !useLegacyMacrosWhenDirectDisabled || bridge == null)
        {
            return;
        }

        if (allowLegacyWholeArmMacros && keys.lowerPressed)
        {
            SendLegacy(prepareToGrabCommand, "2: lower and open");
        }
        if (allowLegacyWholeArmMacros && keys.raisePressed)
        {
            SendLegacy(resetArmCommand, "3: initial raised pose");
        }
        if (keys.closePressed || keys.grabPressed)
        {
            SendLegacy(closeClawCommand, "Z/Space: close claw");
        }
        if (keys.openPressed || keys.releasePressed)
        {
            SendLegacy(openClawCommand, "C/X: open claw");
        }
    }

    void WarnIfDirectServoControlIsRequired(
        float wholeArmInput,
        float elbowInput,
        float rotationInput,
        float clawInput,
        KeyState keys
    )
    {
        if (publishDirectServoCommands || warnedDirectServoDisabled)
        {
            return;
        }

        bool independentJointKey = !Mathf.Approximately(elbowInput, 0f) ||
                                   !Mathf.Approximately(rotationInput, 0f);
        bool armNeedsDirect = !allowLegacyWholeArmMacros &&
                              !Mathf.Approximately(wholeArmInput, 0f);
        bool clawNeedsDirect = !useLegacyMacrosWhenDirectDisabled &&
                               (!Mathf.Approximately(clawInput, 0f) ||
                                keys.grabPressed || keys.releasePressed ||
                                keys.cubeGripPressed);

        if (!independentJointKey && !armNeedsDirect && !clawNeedsDirect)
        {
            return;
        }

        warnedDirectServoDisabled = true;
        Debug.LogWarning(
            "Physical joint control is disabled. Enable Publish Direct Servo Commands " +
            "and run a /cmd_servo subscriber on the rover. Legacy /cmd_gripper cannot " +
            "control S1, S3 or partial joint angles.",
            this
        );
    }

    void SendLegacy(int command, string description)
    {
        bridge.PublishGripperCommand(command);
        if (logCommands)
        {
            Debug.Log($"ROSArmKeyboardTeleop: {description}; /cmd_gripper={command}", this);
        }
    }

    [ContextMenu("Log All Servo Angles")]
    public void LogAllServoAngles()
    {
        LogServoSnapshot(Application.isPlaying ? "Inspector/P context command" : "Edit mode");
        PrimeServoLogCache();
    }

    void LogServoSnapshot(string source)
    {
        ServoJoint[] joints = GetServoJoints();
        StringBuilder message = new StringBuilder(640);
        message.Append("[SERVO SNAPSHOT] ").Append(source)
            .Append(" | directROS=").Append(publishDirectServoCommands)
            .Append(" bridge=").Append(bridge != null ? "assigned" : "missing")
            .AppendLine();

        foreach (ServoJoint joint in joints)
        {
            AppendServoLine(message, joint);
        }

        message.Append("S4 presets: open=").Append(clawOpenAngle.ToString("F1"))
            .Append(" cube=").Append(clawCubeAngle.ToString("F1"))
            .Append(" ball=").Append(clawBallAngle.ToString("F1"));
        Debug.Log(message.ToString(), this);
    }

    void LogChangedServoAngles()
    {
        if (!logServoChanges || Time.unscaledTime < nextServoLogTime)
        {
            return;
        }

        ServoJoint[] joints = GetServoJoints();
        if (!servoLogCacheInitialized)
        {
            PrimeServoLogCache();
            return;
        }

        StringBuilder message = null;
        for (int i = 0; i < joints.Length; i++)
        {
            ServoJoint joint = joints[i];
            float angle = joint.DisplayCurrentAngle;
            if (Mathf.Abs(angle - lastLoggedServoAngles[i]) < servoLogThresholdDegrees)
            {
                continue;
            }

            if (message == null)
            {
                message = new StringBuilder(320);
                message.AppendLine("[SERVO CHANGE] commanded angles");
            }

            AppendServoLine(message, joint);
            lastLoggedServoAngles[i] = angle;
        }

        if (message != null)
        {
            Debug.Log(message.ToString().TrimEnd(), this);
        }

        nextServoLogTime = Time.unscaledTime + servoLogIntervalSeconds;
    }

    void PrimeServoLogCache()
    {
        ServoJoint[] joints = GetServoJoints();
        for (int i = 0; i < joints.Length; i++)
        {
            lastLoggedServoAngles[i] = joints[i].DisplayCurrentAngle;
        }
        servoLogCacheInitialized = true;
        nextServoLogTime = 0f;
    }

    static void AppendServoLine(StringBuilder message, ServoJoint joint)
    {
        message.Append('S').Append(joint.channel)
            .Append(joint.enabled ? " ON  " : " OFF ")
            .Append(joint.label)
            .Append(" | min=").Append(joint.LowAngle.ToString("F1"))
            .Append(" max=").Append(joint.HighAngle.ToString("F1"))
            .Append(" start=").Append(joint.startAngle.ToString("F1"))
            .Append(" current=").Append(joint.DisplayCurrentAngle.ToString("F1"))
            .Append(" target=").Append(joint.DisplayTargetAngle.ToString("F1"))
            .Append(" speed=").Append(joint.degreesPerSecond.ToString("F1"))
            .Append(" deg/s invert=").Append(joint.invertInput)
            .AppendLine();
    }

    ServoJoint[] GetServoJoints()
    {
        return new[]
        {
            armRotationServo,
            shoulderServo,
            elbowServo,
            clawServo,
            cameraTiltServo,
            cameraPanServo
        };
    }

    void ResetAll(bool publish)
    {
        armRotationServo.ResetTarget();
        shoulderServo.ResetTarget();
        elbowServo.ResetTarget();
        clawServo.ResetTarget();
        cameraPanServo.ResetTarget();
        cameraTiltServo.ResetTarget();

        jawClosure01 = 0f;
        wristAngle = unityWristMinAngle;
        cameraPanAngle = 0f;
        cameraTiltAngle = 0f;
        armWasMoving = false;
        elbowWasMoving = false;

        if (mirrorToUnity && gripper != null)
        {
            gripper.ResetState();
        }
        if (cameraPoseCaptured)
        {
            cameraPivot.localRotation = cameraBaseRotation;
        }

        if (publish && publishDirectServoCommands)
        {
            nextDirectServoPublishTime = 0f;
            PublishDirtyServos();
        }
        else if (publish && useLegacyMacrosWhenDirectDisabled && bridge != null)
        {
            SendLegacy(resetArmCommand, "1/R: reset arm");
        }
    }

    void InitializeServoSettings()
    {
        RestoreWorkingCalibrationOnce();
        ValidateServoCalibration();

        if (enforceSafeElbowLimit && shoulderServo != null &&
            shoulderServo.minAngle < minimumSafeElbowAngle)
        {
            if (logCommands)
            {
                Debug.LogWarning(
                    $"ROSArmKeyboardTeleop: S2 minimum {shoulderServo.minAngle:F0} was unsafe; " +
                    $"runtime minimum raised to {minimumSafeElbowAngle:F0} degrees.",
                    this
                );
            }
            shoulderServo.minAngle = minimumSafeElbowAngle;
        }

        armRotationServo.Initialize();
        shoulderServo.Initialize();
        elbowServo.Initialize();
        clawServo.Initialize();
        cameraPanServo.Initialize();
        cameraTiltServo.Initialize();
    }

    [ContextMenu("Restore Last Working Servo Calibration")]
    public void ApplyRecommendedServoCalibration()
    {
        publishDirectServoCommands = true;
        useLegacyMacrosWhenDirectDisabled = false;
        allowLegacyWholeArmMacros = false;
        useDirectCameraServos = true;

        ConfigureJoint(armRotationServo, "Main arm (2/3, S1)", 1, 70f, 110f, 90f, 8f);
        ConfigureJoint(shoulderServo, "Elbow (F/G, S2)", 2, 120f, 160f, 160f, 8f);
        ConfigureJoint(elbowServo, "Claw rotation (Q/E, S3)", 3, 90f, 120f, 90f, 8f);
        ConfigureJoint(clawServo, "Claw grip presets (S4)", 4, 50f, 70f, 50f, 8f);
        ConfigureJoint(cameraTiltServo, "Camera vertical (I/K, S7)", 7, 70f, 110f, 90f, 15f);
        ConfigureJoint(cameraPanServo, "Camera horizontal (J/L, S8)", 8, 15f, 60f, 20f, 15f);
        cameraTiltServo.invertInput = true;
        cameraPanServo.invertInput = false;

        clawOpenAngle = 50f;
        clawCubeAngle = 60f;
        clawBallAngle = 70f;

        servoCalibrationVersion = CurrentServoCalibrationVersion;
        ValidateServoCalibration();
    }

    void RestoreWorkingCalibrationOnce()
    {
        if (servoCalibrationVersion >= CurrentServoCalibrationVersion)
        {
            return;
        }

        if (servoCalibrationVersion <= 0)
        {
            ApplyRecommendedServoCalibration();
            return;
        }

        ApplyExpandedCameraCalibration();
    }

    [ContextMenu("Apply Wider Camera Range")]
    public void ApplyExpandedCameraCalibration()
    {
        ConfigureJoint(cameraTiltServo, "Camera vertical (I/K, S7)", 7, 70f, 110f, 90f, 15f);
        ConfigureJoint(cameraPanServo, "Camera horizontal (J/L, S8)", 8, 15f, 60f, 20f, 15f);
        cameraTiltServo.invertInput = true;
        cameraPanServo.invertInput = false;
        servoCalibrationVersion = CurrentServoCalibrationVersion;
        ValidateServoCalibration();
    }

    public void ValidateServoCalibration()
    {
        armRotationServo?.ValidateSettings();
        shoulderServo?.ValidateSettings();
        elbowServo?.ValidateSettings();
        clawServo?.ValidateSettings();
        cameraTiltServo?.ValidateSettings();
        cameraPanServo?.ValidateSettings();

        if (clawServo != null)
        {
            clawOpenAngle = Mathf.Clamp(clawOpenAngle, clawServo.LowAngle, clawServo.HighAngle);
            clawCubeAngle = Mathf.Clamp(clawCubeAngle, clawServo.LowAngle, clawServo.HighAngle);
            clawBallAngle = Mathf.Clamp(clawBallAngle, clawServo.LowAngle, clawServo.HighAngle);
        }
    }

    public void MoveAllServosToStart()
    {
        ResetAll(true);
    }

    void OnValidate()
    {
        RestoreWorkingCalibrationOnce();
        ValidateServoCalibration();
    }

    static void ConfigureJoint(
        ServoJoint joint,
        string label,
        int channel,
        float minAngle,
        float maxAngle,
        float startAngle,
        float degreesPerSecond
    )
    {
        joint.enabled = true;
        joint.label = label;
        joint.channel = channel;
        joint.minAngle = minAngle;
        joint.maxAngle = maxAngle;
        joint.startAngle = startAngle;
        joint.degreesPerSecond = degreesPerSecond;
    }

    void ConfigureBridgePublishers()
    {
        if (bridge == null)
        {
            return;
        }

        bridge.enableDirectServoTopic = publishDirectServoCommands;
        bridge.enableGripperTopic = useLegacyMacrosWhenDirectDisabled;
        bool useCameraTopics = publishCameraCommands &&
                               (!useDirectCameraServos || publishLegacyVerticalCameraFallback);
        bridge.enableCameraTopic = useCameraTopics;
        bridge.enableCameraTiltTopic = useCameraTopics;
    }

    KeyState ReadKeys()
    {
        KeyState state = new KeyState();
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            return state;
        }

        state.keyboardAvailable = true;
        state.resetPressed = keyboard.digit1Key.wasPressedThisFrame ||
                             keyboard.rKey.wasPressedThisFrame;
        state.lowerHeld = keyboard.digit2Key.isPressed;
        state.lowerPressed = keyboard.digit2Key.wasPressedThisFrame;
        state.raiseHeld = keyboard.digit3Key.isPressed;
        state.raisePressed = keyboard.digit3Key.wasPressedThisFrame;
        state.bendElbowHeld = keyboard.fKey.isPressed;
        state.straightenElbowHeld = keyboard.gKey.isPressed;
        state.rotateLeftHeld = keyboard.qKey.isPressed;
        state.rotateRightHeld = keyboard.eKey.isPressed;
        state.closeHeld = keyboard.zKey.isPressed;
        state.closePressed = keyboard.zKey.wasPressedThisFrame;
        state.openHeld = keyboard.cKey.isPressed;
        state.openPressed = keyboard.cKey.wasPressedThisFrame;
        state.cubeGripPressed = keyboard.vKey.wasPressedThisFrame;
        state.grabPressed = keyboard.spaceKey.wasPressedThisFrame;
        state.releasePressed = keyboard.xKey.wasPressedThisFrame;
        state.cameraLeftHeld = keyboard.jKey.isPressed;
        state.cameraRightHeld = keyboard.lKey.isPressed;
        state.cameraUpHeld = keyboard.iKey.isPressed;
        state.cameraDownHeld = keyboard.kKey.isPressed;
        state.logSnapshotPressed = keyboard.pKey.wasPressedThisFrame;
#else
        state.keyboardAvailable = true;
        state.resetPressed = Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.R);
        state.lowerHeld = Input.GetKey(KeyCode.Alpha2);
        state.lowerPressed = Input.GetKeyDown(KeyCode.Alpha2);
        state.raiseHeld = Input.GetKey(KeyCode.Alpha3);
        state.raisePressed = Input.GetKeyDown(KeyCode.Alpha3);
        state.bendElbowHeld = Input.GetKey(KeyCode.F);
        state.straightenElbowHeld = Input.GetKey(KeyCode.G);
        state.rotateLeftHeld = Input.GetKey(KeyCode.Q);
        state.rotateRightHeld = Input.GetKey(KeyCode.E);
        state.closeHeld = Input.GetKey(KeyCode.Z);
        state.closePressed = Input.GetKeyDown(KeyCode.Z);
        state.openHeld = Input.GetKey(KeyCode.C);
        state.openPressed = Input.GetKeyDown(KeyCode.C);
        state.cubeGripPressed = Input.GetKeyDown(KeyCode.V);
        state.grabPressed = Input.GetKeyDown(KeyCode.Space);
        state.releasePressed = Input.GetKeyDown(KeyCode.X);
        state.cameraLeftHeld = Input.GetKey(KeyCode.J);
        state.cameraRightHeld = Input.GetKey(KeyCode.L);
        state.cameraUpHeld = Input.GetKey(KeyCode.I);
        state.cameraDownHeld = Input.GetKey(KeyCode.K);
        state.logSnapshotPressed = Input.GetKeyDown(KeyCode.P);
#endif
        return state;
    }
}
