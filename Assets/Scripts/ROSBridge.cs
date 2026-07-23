using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Single Unity-to-ROS control path for GFS-X.
/// It publishes geometry_msgs/Twist to /cmd_vel, matching the reference robot node.
/// </summary>
[DisallowMultipleComponent]
public class ROSBridge : MonoBehaviour
{
    [Header("Drive topics")]
    public string velocityTopic = "/cmd_vel";
    [Min(0.01f)] public float maxLinearSpeed = 0.5f;
    [Min(0.01f)] public float maxAngularSpeed = 0.6f;
    [Range(0.1f, 1f)] public float emaAlpha = 0.8f;

    [Header("Manual control")]
    [Tooltip("Use WASD or arrow keys to drive through this bridge.")]
    public bool manualTeleopEnabled = true;
    [Min(1f)] public float manualPublishRateHz = 20f;
    public bool previewInUnity = true;
    public TrackController unityTracks;
    [Tooltip("Maps Unity input to the robot frame: A/D stay unchanged, W/S are reversed.")]
    public bool rotateCommandFrame90 = true;
    public bool disableConflictingControllers = true;
    public bool disableAgentsWhenManual = true;

    [Header("Safety")]
    [Min(0.1f)] public float commandTimeout = 0.5f;
    public bool sendStopOnDisable = true;
    public bool logPublishedCommands = true;
    [Min(0.1f)] public float logInterval = 1f;

    [Header("Optional hardware topics")]
    public bool enableGripperTopic;
    public string gripperTopic = "/cmd_gripper";
    public int closeGripperCommand = 2;
    [Tooltip("Command 4 opens only the claw. Command 1 also lowers the physical arm.")]
    public int openGripperCommand = 4;

    public bool enableCameraTopic;
    public string cameraTopic = "/cmd_camera_pan";
    public bool enableCameraTiltTopic;
    public string cameraTiltTopic = "/cmd_camera_tilt";
    [Tooltip("Enable when the rover's existing /cmd_camera_pan subscriber moves the camera vertically.")]
    public bool swapCameraTopics = true;
    [Tooltip("Safe horizontal offset from the physical servo center.")]
    [Range(1f, 90f)] public float cameraPanLimitDegrees = 20f;
    [Tooltip("Safe vertical offset from the physical servo center.")]
    [Range(1f, 90f)] public float cameraTiltLimitDegrees = 12f;
    [Tooltip("Maximum change sent in one ROS message. One degree gives small servo steps.")]
    [Range(0.1f, 15f)] public float cameraCommandStepDegrees = 1f;

    [Header("Direct servo topic")]
    [Tooltip("Publishes channel and absolute angle as geometry_msgs/Vector3: x=channel, y=angle.")]
    public bool enableDirectServoTopic;
    public string directServoTopic = "/cmd_servo";

    [Header("Target-aware claw")]
    [Tooltip("Use calibrated S4 angles instead of the legacy command 2 for object pickup.")]
    public bool useDirectServoForTargetGrip = true;
    [Range(1, 8)] public int clawServoChannel = 4;
    [Range(15f, 160f)] public float clawOpenAngle = 50f;
    [Range(15f, 160f)] public float clawCubeGripAngle = 60f;
    [Range(15f, 160f)] public float clawBallGripAngle = 70f;

    [Header("Target grip diagnostics (read only)")]
    [SerializeField] int lastGripTargetClassId = -1;
    [SerializeField] float lastGripAngle;
    [SerializeField] int targetGripCommandsSent;

    ROSConnection ros;
    float smoothLinear;
    float smoothAngular;
    float lastCommandTime;
    float nextLogTime;
    float nextServoLogTime;
    float nextManualPublishTime;
    bool publishersRegistered;
    bool keyboardDetected;
    float lastInputLinear;
    float lastInputAngular;
    string inputSource = "none";
    float sentCameraPanDegrees;
    float sentCameraTiltDegrees;

    void Awake()
    {
        ros = ROSConnection.GetOrCreateInstance();

        if (useDirectServoForTargetGrip)
        {
            enableDirectServoTopic = true;
        }

        if (unityTracks == null)
        {
            unityTracks = GetComponent<TrackController>();
        }

        if (manualTeleopEnabled && disableConflictingControllers)
        {
            DisableIfPresent<KeyboardTrackInput>();
            DisableIfPresent<ROSKeyboardTeleop>();
            DisableIfPresent<SimpleWASDDrive>();
            DisableIfPresent<GFSXRealRobotTeleop>();

            if (disableAgentsWhenManual)
            {
                DisableIfPresent<RobotBrain>();
                DisableIfPresent<XiaoAgent>();
            }
        }
    }

    void Start()
    {
        RegisterPublishers();
        PublishStop();
    }

    void Update()
    {
        if (manualTeleopEnabled)
        {
            UpdateManualTeleop();
        }

        if (Time.unscaledTime - lastCommandTime > commandTimeout)
        {
            PublishStop();
        }
    }

    void OnDisable()
    {
        if (sendStopOnDisable)
        {
            PublishStop();
        }
    }

    void OnApplicationQuit()
    {
        PublishStop();
    }

    void UpdateManualTeleop()
    {
        float linear;
        float angular;
        ReadManualInput(out linear, out angular);

        lastInputLinear = linear;
        lastInputAngular = angular;

        if (previewInUnity && unityTracks != null)
        {
            unityTracks.SetCommand(linear, angular);
        }

        if (Time.unscaledTime >= nextManualPublishTime)
        {
            PublishDrive(linear, angular);
            nextManualPublishTime = Time.unscaledTime + 1f / manualPublishRateHz;
        }
    }

    void ReadManualInput(out float linear, out float angular)
    {
        linear = 0f;
        angular = 0f;
        keyboardDetected = false;
        inputSource = "none";

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        keyboardDetected = keyboard != null;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) linear += 1f;
            if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) linear -= 1f;
            if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) angular += 1f;
            if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) angular -= 1f;

            if (!Mathf.Approximately(linear, 0f) || !Mathf.Approximately(angular, 0f))
            {
                inputSource = "Input System keyboard";
            }
        }
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
        if (Mathf.Approximately(linear, 0f) && Mathf.Approximately(angular, 0f))
        {
            linear = Input.GetAxisRaw("Vertical");
            angular = -Input.GetAxisRaw("Horizontal");
            if (!Mathf.Approximately(linear, 0f) || !Mathf.Approximately(angular, 0f))
            {
                inputSource = "Legacy input axes";
            }
        }
#endif
    }

    public void PublishDrive(float normalizedLinear, float normalizedAngular)
    {
        PublishDriveInternal(normalizedLinear, normalizedAngular, rotateCommandFrame90);
    }

    void PublishDriveInternal(
        float normalizedLinear,
        float normalizedAngular,
        bool remapManualCommandFrame)
    {
        RegisterPublishers();

        if (remapManualCommandFrame)
        {
            float inputLinear = normalizedLinear;
            normalizedLinear = normalizedAngular;
            normalizedAngular = -inputLinear;
        }

        normalizedLinear = Mathf.Clamp(normalizedLinear, -1f, 1f);
        normalizedAngular = Mathf.Clamp(normalizedAngular, -1f, 1f);
        lastCommandTime = Time.unscaledTime;

        if (Mathf.Approximately(normalizedLinear, 0f) && Mathf.Approximately(normalizedAngular, 0f))
        {
            smoothLinear = 0f;
            smoothAngular = 0f;
        }
        else
        {
            smoothLinear = Mathf.Lerp(smoothLinear, normalizedLinear, emaAlpha);
            smoothAngular = Mathf.Lerp(smoothAngular, normalizedAngular, emaAlpha);
        }

        PublishTwist(smoothLinear * maxLinearSpeed, smoothAngular * maxAngularSpeed);
    }

    public void PublishCommand(float normalizedLinear, float normalizedAngular)
    {
        // Agent outputs already use ROS semantics: linear.x is throttle and
        // angular.z is steering. Manual keyboard frame correction must not
        // swap these channels during autonomous inference.
        PublishDriveInternal(normalizedLinear, normalizedAngular, false);
    }

    public void PublishStop()
    {
        if (!publishersRegistered)
        {
            return;
        }

        smoothLinear = 0f;
        smoothAngular = 0f;
        lastCommandTime = Time.unscaledTime;
        PublishTwist(0f, 0f);
    }

    public void CloseGripper()
    {
        PublishGripperCommand(closeGripperCommand);
    }

    public void OpenGripper()
    {
        PublishGripperCommand(openGripperCommand);
    }

    public void CloseGripperForTargetClass(int targetClassId)
    {
        if (useDirectServoForTargetGrip)
        {
            enableDirectServoTopic = true;
            float angle = targetClassId == 1
                ? clawCubeGripAngle
                : clawBallGripAngle;
            lastGripTargetClassId = targetClassId;
            lastGripAngle = angle;
            targetGripCommandsSent++;
            PublishServoAngle(clawServoChannel, angle);
            if (logPublishedCommands)
            {
                string targetName = targetClassId == 1 ? "cube" : "ball";
                Debug.Log(
                    $"[ROSBridge] GRAB {targetName}: S{clawServoChannel}={angle:F1} deg",
                    this
                );
            }
            return;
        }

        CloseGripper();
    }

    public void OpenCalibratedClaw()
    {
        if (useDirectServoForTargetGrip)
        {
            PublishServoAngle(clawServoChannel, clawOpenAngle);
            return;
        }

        OpenGripper();
    }

    public void PublishCameraPan(float yawDegrees)
    {
        if (!enableCameraTopic)
        {
            return;
        }

        RegisterPublishers();
        sentCameraPanDegrees = MoveCameraCommand(
            sentCameraPanDegrees,
            yawDegrees,
            cameraPanLimitDegrees
        );
        string topic = swapCameraTopics ? cameraTiltTopic : cameraTopic;
        ros.Publish(topic, new Float32Msg(DegreesToCameraProtocol(sentCameraPanDegrees)));
    }

    public void PublishCameraTilt(float pitchDegrees)
    {
        if (!enableCameraTiltTopic)
        {
            return;
        }

        RegisterPublishers();
        sentCameraTiltDegrees = MoveCameraCommand(
            sentCameraTiltDegrees,
            pitchDegrees,
            cameraTiltLimitDegrees
        );
        string topic = swapCameraTopics ? cameraTopic : cameraTiltTopic;
        ros.Publish(topic, new Float32Msg(DegreesToCameraProtocol(sentCameraTiltDegrees)));
    }

    public void PublishCameraCmd(float normalizedYaw)
    {
        PublishCameraPan(Mathf.Clamp(normalizedYaw, -1f, 1f) * 90f);
    }

    public void PublishGripperCommand(int command)
    {
        if (!enableGripperTopic)
        {
            return;
        }

        RegisterPublishers();
        ros.Publish(gripperTopic, new Int32Msg(command));
    }

    public void PublishGripperCmd(int command)
    {
        PublishGripperCommand(command);
    }

    public void PublishServoAngle(int channel, float angleDegrees)
    {
        if (!enableDirectServoTopic)
        {
            return;
        }

        RegisterPublishers();
        Vector3Msg command = new Vector3Msg();
        command.x = Mathf.Clamp(channel, 1, 8);
        command.y = Mathf.Clamp(angleDegrees, 15f, 160f);
        command.z = 0d;
        ros.Publish(directServoTopic, command);

        if (logPublishedCommands && Time.unscaledTime >= nextServoLogTime)
        {
            Debug.Log(
                $"ROSBridge -> {directServoTopic}: S{(int)command.x}={command.y:F1} deg",
                this
            );
            nextServoLogTime = Time.unscaledTime + logInterval;
        }
    }

    void PublishTwist(float linearX, float angularZ)
    {
        TwistMsg command = new TwistMsg();
        command.linear.x = linearX;
        command.angular.z = angularZ;
        ros.Publish(velocityTopic, command);

        if (logPublishedCommands && Time.unscaledTime >= nextLogTime)
        {
            Debug.Log($"ROSBridge -> {velocityTopic}: linear.x={linearX:F2}, angular.z={angularZ:F2}, input={inputSource}");
            nextLogTime = Time.unscaledTime + logInterval;
        }
    }

    void RegisterPublishers()
    {
        if (publishersRegistered)
        {
            return;
        }

        ros.RegisterPublisher<TwistMsg>(velocityTopic);

        if (enableGripperTopic)
        {
            ros.RegisterPublisher<Int32Msg>(gripperTopic);
        }

        if (enableCameraTopic)
        {
            ros.RegisterPublisher<Float32Msg>(cameraTopic);
        }

        if (enableCameraTiltTopic)
        {
            ros.RegisterPublisher<Float32Msg>(cameraTiltTopic);
        }

        // Register this unconditionally. Autonomous pickup may enable direct
        // S4 control after another publisher has already been registered.
        ros.RegisterPublisher<Vector3Msg>(directServoTopic);

        publishersRegistered = true;
    }

    float MoveCameraCommand(float currentDegrees, float requestedDegrees, float limitDegrees)
    {
        float target = Mathf.Clamp(requestedDegrees, -Mathf.Abs(limitDegrees), Mathf.Abs(limitDegrees));
        return Mathf.MoveTowards(currentDegrees, target, Mathf.Max(0.1f, cameraCommandStepDegrees));
    }

    static float DegreesToCameraProtocol(float degrees)
    {
        // unity_master_team1.py expects -1..1 and converts it to 0..180 around 90 degrees.
        return Mathf.Clamp(degrees / 90f, -1f, 1f);
    }

    void DisableIfPresent<T>() where T : Behaviour
    {
        T controller = GetComponent<T>();
        if (controller != null)
        {
            controller.enabled = false;
        }
    }
}
