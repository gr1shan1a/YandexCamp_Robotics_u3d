using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Manual teleoperation of the physical robot through ROSBridge.
/// It is intentionally separate from ML-Agents and the local Unity movement scripts.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(ROSBridge))]
public class ROSKeyboardTeleop : MonoBehaviour
{
    public ROSBridge bridge;
    public bool enableTeleoperation = true;
    [Min(1f)] public float publishRateHz = 20f;
    public bool enableGripperKeys;

    [Header("Unity preview")]
    public bool driveUnityPreview = true;
    public TrackController unityTracks;

    [Header("Diagnostics")]
    public bool showDebugHud = true;
    public bool logInputState = true;
    [Min(0.1f)] public float debugLogInterval = 1f;

    float nextPublishTime;
    float nextDebugLogTime;
    float lastLinearInput;
    float lastAngularInput;
    bool keyboardDetected;
    bool gameFocused;

    void Awake()
    {
        if (bridge == null)
        {
            bridge = GetComponent<ROSBridge>();
        }

        if (unityTracks == null)
        {
            unityTracks = GetComponent<TrackController>();
        }
    }

    void Update()
    {
        if (!enableTeleoperation || bridge == null)
        {
            return;
        }

        float linear = 0f;
        float angular = 0f;
        keyboardDetected = false;
        gameFocused = Application.isFocused;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        keyboardDetected = keyboard != null;
        if (keyboard == null)
        {
            lastLinearInput = 0f;
            lastAngularInput = 0f;
            ReportInputState();
            return;
        }

        if (keyboard.wKey.isPressed) linear += 1f;
        if (keyboard.sKey.isPressed) linear -= 1f;
        if (keyboard.aKey.isPressed) angular += 1f;
        if (keyboard.dKey.isPressed) angular -= 1f;

        if (enableGripperKeys && keyboard.spaceKey.wasPressedThisFrame)
        {
            bridge.CloseGripper();
        }

        if (enableGripperKeys && keyboard.leftShiftKey.wasPressedThisFrame)
        {
            bridge.OpenGripper();
        }
#elif ENABLE_LEGACY_INPUT_MANAGER
        keyboardDetected = true;
        linear = Input.GetAxisRaw("Vertical");
        angular = -Input.GetAxisRaw("Horizontal");

        if (enableGripperKeys && Input.GetKeyDown(KeyCode.Space))
        {
            bridge.CloseGripper();
        }

        if (enableGripperKeys && Input.GetKeyDown(KeyCode.LeftShift))
        {
            bridge.OpenGripper();
        }
#endif

        lastLinearInput = linear;
        lastAngularInput = angular;
        ReportInputState();

        if (driveUnityPreview && unityTracks != null)
        {
            unityTracks.SetCommand(linear, angular);
        }

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

        if (unityTracks != null)
        {
            unityTracks.SetCommand(0f, 0f);
        }
    }

    void ReportInputState()
    {
        if (!logInputState || Time.unscaledTime < nextDebugLogTime)
        {
            return;
        }

        Debug.Log(
            $"ROSKeyboardTeleop: focused={gameFocused}, keyboard={keyboardDetected}, " +
            $"linear={lastLinearInput:F1}, angular={lastAngularInput:F1}"
        );
        nextDebugLogTime = Time.unscaledTime + debugLogInterval;
    }

    void OnGUI()
    {
        if (!showDebugHud)
        {
            return;
        }

        GUI.Label(
            new Rect(10, 175, 520, 80),
            $"ROS teleop: enabled={enableTeleoperation}  focused={gameFocused}  keyboard={keyboardDetected}\n" +
            $"W/S input={lastLinearInput:F1}  A/D input={lastAngularInput:F1}  preview={driveUnityPreview}"
        );
    }
}
