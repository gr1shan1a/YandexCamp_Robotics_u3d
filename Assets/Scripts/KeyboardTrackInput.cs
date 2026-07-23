using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// One manual control path for both the Unity preview and the physical robot.
/// Enable Publish To ROS only when a ROSBridge is assigned.
/// </summary>
[RequireComponent(typeof(TrackController))]
public class KeyboardTrackInput : MonoBehaviour
{
    public bool enableKeyboard = true;

    [Header("Physical robot")]
    public bool publishToRos;
    public ROSBridge rosBridge;

    [Header("Diagnostics")]
    public bool showDebugHud = true;
    public bool logInputState = true;
    [Min(0.1f)] public float debugLogInterval = 1f;

    [Header("Connection test")]
    [Tooltip("Temporarily sends the values below instead of keyboard input. Turn it off after the test.")]
    public bool useCommandOverride;
    [Range(-1f, 1f)] public float overrideLinear;
    [Range(-1f, 1f)] public float overrideAngular;

    TrackController tracks;
    float lastGas;
    float lastSteer;
    float nextDebugLogTime;
    bool keyboardDetected;

    void Awake()
    {
        tracks = GetComponent<TrackController>();

        if (rosBridge == null)
        {
            rosBridge = GetComponent<ROSBridge>();
        }
    }

    void Update()
    {
        if (!enableKeyboard)
        {
            return;
        }

        float gas = 0f;
        float steer = 0f;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        keyboardDetected = keyboard != null;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) gas += 1f;
            if (keyboard.sKey.isPressed) gas -= 1f;
            if (keyboard.aKey.isPressed) steer -= 1f;
            if (keyboard.dKey.isPressed) steer += 1f;
            if (keyboard.upArrowKey.isPressed) gas += 1f;
            if (keyboard.downArrowKey.isPressed) gas -= 1f;
            if (keyboard.leftArrowKey.isPressed) steer -= 1f;
            if (keyboard.rightArrowKey.isPressed) steer += 1f;
        }
#elif ENABLE_LEGACY_INPUT_MANAGER
        keyboardDetected = true;
        if (Input.GetKey(KeyCode.W)) gas += 1f;
        if (Input.GetKey(KeyCode.S)) gas -= 1f;
        if (Input.GetKey(KeyCode.A)) steer -= 1f;
        if (Input.GetKey(KeyCode.D)) steer += 1f;
#endif

        if (useCommandOverride)
        {
            gas = overrideLinear;
            steer = overrideAngular;
        }

        lastGas = gas;
        lastSteer = steer;
        tracks.SetCommand(gas, steer);

        if (publishToRos && rosBridge != null)
        {
            rosBridge.PublishDrive(gas, steer);
        }

        if (logInputState && Time.unscaledTime >= nextDebugLogTime)
        {
            Debug.Log(
                $"KeyboardTrackInput: focused={Application.isFocused}, keyboard={keyboardDetected}, " +
                $"W/S={gas:F1}, A/D={steer:F1}, override={useCommandOverride}, " +
                $"publishToRos={publishToRos}, bridge={rosBridge != null}"
            );
            nextDebugLogTime = Time.unscaledTime + debugLogInterval;
        }
    }

    void OnDisable()
    {
        if (tracks != null)
        {
            tracks.SetCommand(0f, 0f);
        }

        if (publishToRos && rosBridge != null)
        {
            rosBridge.PublishStop();
        }
    }

    void OnGUI()
    {
        if (!showDebugHud)
        {
            return;
        }

        GUI.Label(
            new Rect(10, 175, 600, 80),
            $"Manual WASD: focused={Application.isFocused}  keyboard={keyboardDetected}\n" +
            $"W/S={lastGas:F1}  A/D={lastSteer:F1}  override={useCommandOverride}\n" +
            $"ROS publish={publishToRos}  bridge={rosBridge != null}"
        );
    }
}
