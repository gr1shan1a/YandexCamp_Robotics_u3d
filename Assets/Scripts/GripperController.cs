using UnityEngine;

public class GripperController : MonoBehaviour
{
    public enum ArmPose
    {
        Idle,
        Reach,
        Carry
    }

    [Header("Logical grab")]
    public Transform holdPoint;
    public VirtualSensors sensors;
    public string targetBallTag = "TargetBall";
    public float grabRadius = 0.12f;
    public bool requireGripperSensor = false;

    [Header("Claws")]
    [Tooltip("Drag upClaw here.")]
    public Transform leftJaw;
    [Tooltip("Drag downClaw here.")]
    public Transform rightJaw;
    public Vector3 rotationAxis = Vector3.right;
    public float closeAngle = 6f;
    public float moveSpeed = 12f;
    public bool invertDirection = false;

    [Header("Optional wrist")]
    public Transform wristRollPivot;
    public Vector3 wristRollAxis = Vector3.forward;
    public float wristRollMaxAngle = 90f;

    public bool IsHolding => heldBall != null;
    public bool HasBall => IsHolding;
    public bool JawsClosed { get; private set; }
    public ArmPose CurrentPose { get; private set; } = ArmPose.Idle;
    public bool PoseReached { get; private set; } = true;

    Quaternion leftOpenRotation = Quaternion.identity;
    Quaternion rightOpenRotation = Quaternion.identity;
    Quaternion wristOpenRotation = Quaternion.identity;
    Quaternion leftTargetRotation = Quaternion.identity;
    Quaternion rightTargetRotation = Quaternion.identity;
    Quaternion wristTargetRotation = Quaternion.identity;

    Rigidbody heldRb;
    Collider heldCollider;
    Transform heldBall;
    Transform originalParent;

    void Awake()
    {
        if (sensors == null)
        {
            sensors = GetComponentInParent<VirtualSensors>();
        }

        if (leftJaw != null)
        {
            leftOpenRotation = leftJaw.localRotation;
            leftTargetRotation = leftOpenRotation;
        }

        if (rightJaw != null)
        {
            rightOpenRotation = rightJaw.localRotation;
            rightTargetRotation = rightOpenRotation;
        }

        if (wristRollPivot != null)
        {
            wristOpenRotation = wristRollPivot.localRotation;
            wristTargetRotation = wristOpenRotation;
        }
    }

    void Update()
    {
        if (leftJaw != null)
        {
            leftJaw.localRotation = Quaternion.Slerp(
                leftJaw.localRotation,
                leftTargetRotation,
                moveSpeed * Time.deltaTime
            );
        }

        if (rightJaw != null)
        {
            rightJaw.localRotation = Quaternion.Slerp(
                rightJaw.localRotation,
                rightTargetRotation,
                moveSpeed * Time.deltaTime
            );
        }

        if (wristRollPivot != null)
        {
            wristRollPivot.localRotation = Quaternion.Slerp(
                wristRollPivot.localRotation,
                wristTargetRotation,
                moveSpeed * Time.deltaTime
            );
        }
    }

    public void SetGripper(float command)
    {
        if (command > 0f)
        {
            Close();
        }
        else if (command < -0.25f)
        {
            Open();
        }
    }

    public void CloseJaws()
    {
        JawsClosed = true;
        float sign = invertDirection ? -1f : 1f;
        Vector3 axis = rotationAxis.sqrMagnitude > 0.0001f ? rotationAxis.normalized : Vector3.right;

        leftTargetRotation = leftOpenRotation * Quaternion.AngleAxis(closeAngle * sign, axis);
        rightTargetRotation = rightOpenRotation * Quaternion.AngleAxis(-closeAngle * sign, axis);
    }

    public void OpenJaws()
    {
        JawsClosed = false;
        leftTargetRotation = leftOpenRotation;
        rightTargetRotation = rightOpenRotation;
    }

    public void Close()
    {
        CloseJaws();
        TryGrab();
    }

    public void Open()
    {
        OpenJaws();
        Release();
    }

    public void ResetState()
    {
        Release();
        OpenJaws();
        SetWristRoll(0f);
        CurrentPose = ArmPose.Idle;
        PoseReached = true;
    }

    public void SetPose(ArmPose pose)
    {
        CurrentPose = pose;
        PoseReached = true;
    }

    public void SetWristRoll(float roll)
    {
        if (wristRollPivot == null)
        {
            return;
        }

        float clampedRoll = Mathf.Clamp(roll, -1f, 1f);
        Vector3 axis = wristRollAxis.sqrMagnitude > 0.0001f ? wristRollAxis.normalized : Vector3.forward;
        wristTargetRotation = wristOpenRotation * Quaternion.AngleAxis(clampedRoll * wristRollMaxAngle, axis);
    }

    public void Release()
    {
        if (!IsHolding)
        {
            return;
        }

        heldBall.SetParent(originalParent);

        if (heldCollider != null)
        {
            heldCollider.enabled = true;
        }

        if (heldRb != null)
        {
            heldRb.isKinematic = false;
            heldRb.linearVelocity = Vector3.zero;
            heldRb.angularVelocity = Vector3.zero;
        }

        heldBall = null;
        heldRb = null;
        heldCollider = null;
        originalParent = null;
    }

    void TryGrab()
    {
        if (IsHolding || holdPoint == null)
        {
            return;
        }

        if (requireGripperSensor && (sensors == null || sensors.GripperIR < 0.5f))
        {
            return;
        }

        Collider[] hits = Physics.OverlapSphere(holdPoint.position, grabRadius);
        foreach (Collider hit in hits)
        {
            if (hit.gameObject.tag != targetBallTag)
            {
                continue;
            }

            heldBall = hit.transform;
            originalParent = heldBall.parent;
            heldRb = heldBall.GetComponent<Rigidbody>();
            heldCollider = hit;

            if (heldRb != null)
            {
                heldRb.isKinematic = true;
                heldRb.linearVelocity = Vector3.zero;
                heldRb.angularVelocity = Vector3.zero;
            }

            heldCollider.enabled = false;
            heldBall.SetParent(holdPoint);
            heldBall.localPosition = Vector3.zero;
            heldBall.localRotation = Quaternion.identity;
            return;
        }
    }

    void OnDrawGizmosSelected()
    {
        if (holdPoint == null)
        {
            return;
        }

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(holdPoint.position, grabRadius);
    }
}
