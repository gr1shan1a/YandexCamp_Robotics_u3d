using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
public class SimpleWASDDrive : MonoBehaviour
{
    public float moveSpeed = 1.5f;
    public float turnSpeed = 120f;
    public float movementYawOffset = 90f;

    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void FixedUpdate()
    {
        float move = 0f;
        float turn = 0f;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) move += 1f;
            if (keyboard.sKey.isPressed) move -= 1f;
            if (keyboard.aKey.isPressed) turn -= 1f;
            if (keyboard.dKey.isPressed) turn += 1f;
        }
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
        if (Input.GetKey(KeyCode.W)) move += 1f;
        if (Input.GetKey(KeyCode.S)) move -= 1f;
        if (Input.GetKey(KeyCode.A)) turn -= 1f;
        if (Input.GetKey(KeyCode.D)) turn += 1f;
#endif

        Quaternion nextRotation = rb.rotation * Quaternion.Euler(0f, turn * turnSpeed * Time.fixedDeltaTime, 0f);
        Vector3 driveForward = Quaternion.Euler(0f, movementYawOffset, 0f) * transform.forward;
        Vector3 nextPosition = rb.position + driveForward * move * moveSpeed * Time.fixedDeltaTime;

        rb.MoveRotation(nextRotation);
        rb.MovePosition(nextPosition);
    }
}
