using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("Motion")]
    public float moveSpeed = 0.57f;
    public float turnSpeed = 120f;
    public float turnK = 0.30f;
    public float maxLinearCmd = 0.25f;
    public float movementYawOffset = 90f;

    [Header("Motor model")]
    public float motorDeadzone = 10f;
    public float minMotorPwm = 35f;
    public float maxPwmStep = 15f;
    public float velocityToPwm = 200f;

    Rigidbody rb;
    float gasCommand;
    float steerCommand;
    float leftPwm;
    float rightPwm;

    public Vector2 CurrentPwm => new Vector2(leftPwm, rightPwm);

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = 2.5f;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.linearDamping = 8f;
        rb.angularDamping = 8f;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
    }

    public void SetCommand(float gas, float steer)
    {
        gasCommand = Mathf.Clamp(gas, -1f, 1f);
        steerCommand = Mathf.Clamp(steer, -1f, 1f);
    }

    public void Stop()
    {
        gasCommand = 0f;
        steerCommand = 0f;
        leftPwm = 0f;
        rightPwm = 0f;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    void FixedUpdate()
    {
        float linear = Mathf.Clamp(gasCommand * moveSpeed, -maxLinearCmd, maxLinearCmd);
        float turn = steerCommand * turnK;

        float targetLeftPwm = SpeedToPwm(linear - turn);
        float targetRightPwm = SpeedToPwm(linear + turn);

        leftPwm = Mathf.MoveTowards(leftPwm, targetLeftPwm, maxPwmStep);
        rightPwm = Mathf.MoveTowards(rightPwm, targetRightPwm, maxPwmStep);

        float leftSpeed = PwmToSpeed(leftPwm);
        float rightSpeed = PwmToSpeed(rightPwm);
        float forwardSpeed = (leftSpeed + rightSpeed) * 0.5f;
        float yawSpeed = (rightSpeed - leftSpeed) * turnSpeed;

        Quaternion nextRotation = rb.rotation * Quaternion.Euler(0f, yawSpeed * Time.fixedDeltaTime, 0f);
        Vector3 driveForward = Quaternion.Euler(0f, movementYawOffset, 0f) * transform.forward;
        Vector3 nextPosition = rb.position + driveForward * forwardSpeed * Time.fixedDeltaTime;

        rb.MoveRotation(nextRotation);
        rb.MovePosition(nextPosition);
    }

    float SpeedToPwm(float speed)
    {
        float sign = Mathf.Sign(speed);
        float pwm = Mathf.Abs(speed) * velocityToPwm;

        if (pwm < motorDeadzone)
        {
            return 0f;
        }

        return sign * Mathf.Clamp(Mathf.Max(pwm, minMotorPwm), 0f, 100f);
    }

    float PwmToSpeed(float pwm)
    {
        return Mathf.Sign(pwm) * Mathf.Abs(pwm) / velocityToPwm;
    }
}
