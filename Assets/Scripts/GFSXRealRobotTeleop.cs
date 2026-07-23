using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// The single manual-control path for the Unity preview and the physical GFS-X.
/// It mirrors the reference project's RealRobotBridge: commands are published
/// at a stable frequency through ROSBridge to /cmd_vel.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(ROSBridge))]
public class GFSXRealRobotTeleop : MonoBehaviour
{
    [Header("Control")]
    public bool enableTeleoperation = true;
    [Min(1f)] public float publishRateHz = 20f;
    public bool driveUnityPreview = true;
    public TrackController unityTracks;

    [Header("Scene cleanup")]
    [Tooltip("Disables the older manual scripts so that only this component writes drive commands.")]
    public bool disableOlderManualControllers = true;
    [Tooltip("Disable RobotBrain during a real manual-drive session. Turn this component off before ML training.")]
    public bool disableRobotBrain = true;

    [Header("Screen controls")]
    public bool showControls = true;
    [Range(0.05f, 1f)] public float screenControlPower = 0.35f;

    ROSBridge bridge;
    float nextPublishTime;
    float guiLinear;
    float guiAngular;
    float guiCommandUntil;
    float lastLinear;
    float lastAngular;
    string lastSource = "none";
    bool keyboardAvailable;

    void Awake()
    {
        bridge = GetComponent<ROSBridge>();

        if (unityTracks == null)
        {
            unityTracks = GetComponent<TrackController>();
        }

        if (!disableOlderManualControllers)
        {
            return;
        }

        DisableIfPresent<KeyboardTrackInput>();
        DisableIfPresent<ROSKeyboardTeleop>();
        DisableIfPresent<SimpleWASDDrive>();

        if (disableRobotBrain)
        {
            DisableIfPresent<RobotBrain>();
            DisableIfPresent<XiaoAgent>();
        }
    }

    void Update()
    {
        if (!enableTeleoperation || bridge == null)
        {
            ApplyPreview(0f, 0f);
            return;
        }

        float linear;
        float angular;
        ReadKeyboard(out linear, out angular);

        if (Time.unscaledTime <= guiCommandUntil)
        {
            linear = guiLinear;
            angular = guiAngular;
            lastSource = "screen button";
        }

        lastLinear = linear;
        lastAngular = angular;
        ApplyPreview(linear, angular);

        if (Time.unscaledTime >= nextPublishTime)
        {
            bridge.PublishDrive(linear, angular);
            nextPublishTime = Time.unscaledTime + 1f / publishRateHz;
        }
    }

    void OnDisable()
    {
        if (bridge != null)
        {
            bridge.PublishStop();
        }

        ApplyPreview(0f, 0f);
    }

    void ReadKeyboard(out float linear, out float angular)
    {
        linear = 0f;
        angular = 0f;
        keyboardAvailable = false;
        lastSource = "none";

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        keyboardAvailable = keyboard != null;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) linear += 1f;
            if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) linear -= 1f;
            if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) angular += 1f;
            if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) angular -= 1f;

            if (!Mathf.Approximately(linear, 0f) || !Mathf.Approximately(angular, 0f))
            {
                lastSource = "Input System keyboard";
            }
        }
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
        // The reference project uses these axes. When Active Input Handling is
        // set to Both, they are a fallback for projects with legacy mappings.
        if (Mathf.Approximately(linear, 0f) && Mathf.Approximately(angular, 0f))
        {
            linear = Input.GetAxisRaw("Vertical");
            angular = -Input.GetAxisRaw("Horizontal");
            if (!Mathf.Approximately(linear, 0f) || !Mathf.Approximately(angular, 0f))
            {
                lastSource = "Legacy input axes";
            }
        }
#endif
    }

    void ApplyPreview(float linear, float angular)
    {
        if (driveUnityPreview && unityTracks != null)
        {
            unityTracks.SetCommand(linear, angular);
        }
    }

    void SendScreenCommand(float linear, float angular)
    {
        guiLinear = linear * screenControlPower;
        guiAngular = angular * screenControlPower;
        guiCommandUntil = Time.unscaledTime + 0.15f;
    }

    void OnGUI()
    {
        if (!showControls)
        {
            return;
        }

        GUILayout.BeginArea(new Rect(10, 150, 320, 290), GUI.skin.box);
        GUILayout.Label("GFS-X: manual ROS control");
        GUILayout.Label("Bridge: ROSBridge   Topic: /cmd_vel");
        GUILayout.Label($"Enabled: {enableTeleoperation}   Keyboard: {keyboardAvailable}");
        GUILayout.Label($"Input: {lastSource}   L={lastLinear:F1} A={lastAngular:F1}");
        GUILayout.Label("W/S + A/D or arrows. Hold a screen button as a fallback.");

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        if (GUILayout.RepeatButton("FORWARD", GUILayout.Width(110), GUILayout.Height(32)))
        {
            SendScreenCommand(1f, 0f);
        }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.RepeatButton("LEFT", GUILayout.Width(90), GUILayout.Height(32)))
        {
            SendScreenCommand(0f, 1f);
        }
        if (GUILayout.RepeatButton("STOP", GUILayout.Width(90), GUILayout.Height(32)))
        {
            SendScreenCommand(0f, 0f);
        }
        if (GUILayout.RepeatButton("RIGHT", GUILayout.Width(90), GUILayout.Height(32)))
        {
            SendScreenCommand(0f, -1f);
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        if (GUILayout.RepeatButton("BACK", GUILayout.Width(110), GUILayout.Height(32)))
        {
            SendScreenCommand(-1f, 0f);
        }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        GUILayout.EndArea();
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
