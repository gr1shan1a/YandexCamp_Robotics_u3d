using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(TrackController))]
public class KeyboardTrackInput : MonoBehaviour
{
    public bool enableKeyboard = true;

    TrackController tracks;

    void Awake()
    {
        tracks = GetComponent<TrackController>();
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
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) gas += 1f;
            if (keyboard.sKey.isPressed) gas -= 1f;
            if (keyboard.aKey.isPressed) steer -= 1f;
            if (keyboard.dKey.isPressed) steer += 1f;
        }
#elif ENABLE_LEGACY_INPUT_MANAGER

        if (Input.GetKey(KeyCode.W))
        {
            gas += 1f;
        }

        if (Input.GetKey(KeyCode.S))
        {
            gas -= 1f;
        }

        if (Input.GetKey(KeyCode.A))
        {
            steer -= 1f;
        }

        if (Input.GetKey(KeyCode.D))
        {
            steer += 1f;
        }
#endif

        tracks.SetCommand(gas, steer);
    }
}
