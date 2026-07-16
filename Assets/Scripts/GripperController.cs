using UnityEngine;

public class GripperController : MonoBehaviour
{
    public Transform holdPoint;
    public VirtualSensors sensors;
    public string targetBallTag = "TargetBall";
    public float grabRadius = 0.12f;
    public bool IsHolding => heldBall != null;

    Rigidbody heldRb;
    Collider heldCollider;
    Transform heldBall;
    Transform originalParent;

    public void SetGripper(float command)
    {
        if (command > 0f)
        {
            TryGrab();
        }
        else if (command < -0.25f)
        {
            Release();
        }
    }

    void TryGrab()
    {
        if (IsHolding || sensors == null || sensors.GripperIR < 0.5f || holdPoint == null)
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
            }

            heldCollider.enabled = false;
            heldBall.SetParent(holdPoint);
            heldBall.localPosition = Vector3.zero;
            heldBall.localRotation = Quaternion.identity;
            return;
        }
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
}
