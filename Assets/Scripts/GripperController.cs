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
    [Tooltip("Assign the scene Sphere here. It must have a Rigidbody and Collider.")]
    public Transform targetBall;
    public VirtualSensors sensors;
    public string targetBallTag = "TargetBall";
    public float grabRadius = 0.12f;
    public bool requireGripperSensor = false;
    public bool autoGrabWhenJawsClosed = true;
    public bool snapBallToHoldPoint = true;
    [Tooltip("Safe jaw angle used after the ball is detected. Must be smaller than Close Angle.")]
    public float ballHoldAngle = 2.5f;

    [Header("Arm")]
    [Tooltip("Empty object placed at the shoulder rotation center.")]
    public Transform shoulderPoint;
    [Tooltip("The real model object that contains the complete moving arm.")]
    public Transform armRoot;
    [Tooltip("Pivot of the second parallel shoulder link. Uses Shoulder Point when left empty.")]
    public Transform secondShoulderPoint;
    [Tooltip("The second physical shoulder link that must move together with Arm Root.")]
    public Transform secondArmRoot;
    public bool invertSecondShoulder = false;
    [Tooltip("Shoulder rotation axis in shoulderPoint local coordinates.")]
    public Vector3 shoulderAxis = Vector3.forward;
    public float reachAngle = -25f;
    public float carryAngle = 20f;
    [Tooltip("Additional downward travel available while holding key 2.")]
    public float reachExtraDrop = 5f;
    [Tooltip("Absolute lower limit for the shoulder. This lets the arm reach noticeably closer to the floor.")]
    public float minShoulderDownAngle = -40f;

    [Header("Elbow")]
    [Tooltip("Empty object placed at the elbow hinge. It must follow the shoulder link.")]
    public Transform elbowPoint;
    [Tooltip("Stable upper-arm link that carries Elbow Point. Leave empty to use Forearm Root's parent automatically.")]
    public Transform elbowFollowRoot;
    [Tooltip("The model object containing the forearm, wrist and claws.")]
    public Transform forearmRoot;
    [Tooltip("Use Circle.002's parent (Cube.003) as the elbow frame. Correct for the provided hierarchy.")]
    public bool useForearmParentAsElbowFrame = true;
    [Tooltip("Use the red local X axis of elbowPoint. This bends the arm in the vertical plane.")]
    public bool useElbowLocalRightAxis = true;
    [Tooltip("Used only when Use Elbow Local Right Axis is disabled.")]
    public Vector3 elbowAxis = Vector3.right;
    [Tooltip("Manual bend limit in degrees. Negative and positive values are allowed.")]
    public float elbowMinAngle = -35f;
    [Tooltip("Manual straighten limit in degrees. Negative and positive values are allowed.")]
    public float elbowMaxAngle = 35f;
    public float elbowReachAngle = 35f;
    public float elbowCarryAngle = -20f;
    public float elbowDegreesPerSecond = 20f;
    public float armDegreesPerSecond = 15f;

    [Header("Claws")]
    [Tooltip("Drag upClaw here.")]
    public Transform leftJaw;
    [Tooltip("Drag downClaw here.")]
    public Transform rightJaw;
    [Tooltip("Center between the two jaw gears (rotP). Keep it under Circle.002.")]
    public Transform jawPivotPoint;
    [Tooltip("Jaw rotation axis in the local coordinates of their common parent (Circle.002).")]
    public Vector3 rotationAxis = Vector3.forward;
    [Tooltip("Automatically keeps jaw motion in the vertical arm plane. Recommended for GFS-X.")]
    public bool useVerticalJawPlane = true;
    [Tooltip("Additional angle from the imported pose when the jaws are open.")]
    public float openAngle = 10f;
    [Tooltip("Angle from the imported pose when the jaws are closed.")]
    public float closeAngle = 6f;
    public float jawDegreesPerSecond = 60f;
    public float moveSpeed = 12f;
    public bool invertDirection = false;

    [Header("Optional wrist")]
    [Tooltip("Empty object placed at the wrist rotation center (rotatePoint).")]
    public Transform wristPivotPoint;
    [Tooltip("Stable parent that carries rotatePoint. Assign Cube.003, not Circle.002.")]
    public Transform wristFollowRoot;
    [Tooltip("The complete gripper assembly that rotates around rotatePoint.")]
    public Transform wristRoot;
    [Tooltip("Use rotatePoint local X axis, perpendicular to the previous Z-axis motion.")]
    public bool usePerpendicularWristPlane = true;
    [Tooltip("Build the rotation axis from the elbow to rotatePoint instead of using rotatePoint's tilted Rotation.")]
    public bool alignWristAxisWithArm = true;
    [Tooltip("Optional axis reference. The configured elbow pivot is used automatically when this is empty.")]
    public Transform wristAxisReferencePoint;
    public Vector3 wristRollAxis = Vector3.right;
    public float wristRollMaxAngle = 90f;
    public float wristDegreesPerSecond = 60f;
    [Tooltip("Rotate in the opposite direction around Wrist Roll Axis.")]
    public bool clockwiseWristRotation = true;
    [Tooltip("Calibrates the visual center of asymmetric claws without changing the 90 degree actuator travel.")]
    [Range(-30f, 30f)]
    public float wristAlignmentOffsetDegrees = -5f;

    public bool IsHolding => heldBall != null;
    public bool HasBall => IsHolding;
    public bool JawsClosed { get; private set; }
    public float JawClosure01 { get; private set; }
    public bool ElbowReady => elbowConfigurationValid;
    public float CurrentElbowAngle => currentElbowAngle;
    public ArmPose CurrentPose { get; private set; } = ArmPose.Idle;
    public bool PoseReached { get; private set; } = true;

    Quaternion leftOpenRotation = Quaternion.identity;
    Quaternion rightOpenRotation = Quaternion.identity;
    Quaternion wristOpenRotation = Quaternion.identity;
    Quaternion wristTargetRotation = Quaternion.identity;

    Vector3 wristPivotPositionInFollowRoot;
    Quaternion wristPivotRotationInFollowRoot = Quaternion.identity;
    Vector3 wristAxisReferencePositionInFollowRoot;
    Quaternion wristAxisReferenceRotationInFollowRoot = Quaternion.identity;
    bool wristAxisReferenceCaptured;
    Vector3 wristOpenPositionInPivot;
    Quaternion wristOpenRotationInPivot = Quaternion.identity;
    Transform activeWristFollowRoot;
    bool wristConfigurationValid;
    bool wristUsesForearmFrame;
    float currentWristAngle;
    float targetWristAngle;

    Vector3 leftBasePositionInPivot;
    Vector3 rightBasePositionInPivot;
    Quaternion leftBaseRotationInPivot = Quaternion.identity;
    Quaternion rightBaseRotationInPivot = Quaternion.identity;
    float currentJawAngle;
    float targetJawAngle;

    Vector3 armOpenPositionInPivot;
    Quaternion armOpenRotationInPivot = Quaternion.identity;
    Vector3 shoulderPivotPositionInRobot;
    Quaternion shoulderPivotRotationInRobot = Quaternion.identity;
    Vector3 secondArmOpenPositionInPivot;
    Quaternion secondArmOpenRotationInPivot = Quaternion.identity;
    Vector3 secondPivotPositionInRobot;
    Quaternion secondPivotRotationInRobot = Quaternion.identity;
    Vector3 forearmOpenPositionInPivot;
    Quaternion forearmOpenRotationInPivot = Quaternion.identity;
    Vector3 elbowPivotPositionInFollowRoot;
    Quaternion elbowPivotRotationInFollowRoot = Quaternion.identity;
    Transform activeElbowFollowRoot;
    Transform elbowDriveRoot;
    bool elbowConfigurationValid;
    bool warnedInvalidElbow;
    float currentArmAngle;
    float targetArmAngle;
    float currentElbowAngle;
    float targetElbowAngle;
    bool isPrimaryController;

    Rigidbody heldRb;
    Collider[] heldColliders;
    bool[] heldColliderStates;
    bool heldRbWasKinematic;
    bool heldRbUsedGravity;
    Transform heldBall;
    Transform originalParent;

    void Awake()
    {
        isPrimaryController = FindController(transform.root) == this;
        if (!isPrimaryController)
        {
            enabled = false;
            Debug.LogWarning(
                $"{name}: duplicate GripperController disabled. Keep only the controller nearest to upClaw/downClaw.",
                this
            );
            return;
        }

        if (sensors == null)
        {
            sensors = GetComponentInParent<VirtualSensors>();
        }

        if (holdPoint != null && holdPoint.gameObject.tag == targetBallTag)
        {
            holdPoint.gameObject.tag = "Untagged";
            Debug.LogWarning("Hold Point had the TargetBall tag. It was changed to Untagged at runtime.", holdPoint);
        }

        if (targetBall == null)
        {
            targetBall = FindSceneTargetBall();
        }

        if (leftJaw != null)
        {
            leftOpenRotation = leftJaw.localRotation;
        }

        if (rightJaw != null)
        {
            rightOpenRotation = rightJaw.localRotation;
        }

        // The provided GFS-X model has upper/lower jaws. A single physical drive
        // point (rotP) plus a horizontal shaft gives the expected up/down motion.
        // The older virtual-gear experiment made this imported model open sideways.
        CaptureJawBasePose();

        if (wristRoot != null)
        {
            wristOpenRotation = wristRoot.localRotation;
            wristTargetRotation = wristOpenRotation;

            if (wristPivotPoint != null)
            {
                activeWristFollowRoot = wristFollowRoot != null
                    ? wristFollowRoot
                    : (wristRoot == forearmRoot ? wristRoot : wristRoot.parent);
                wristUsesForearmFrame = activeWristFollowRoot == wristRoot && wristRoot == forearmRoot;
                wristConfigurationValid = activeWristFollowRoot != null &&
                                          (wristUsesForearmFrame ||
                                           (activeWristFollowRoot != wristRoot &&
                                            !activeWristFollowRoot.IsChildOf(wristRoot)));

                if (wristConfigurationValid)
                {
                    CaptureRelativePose(
                        activeWristFollowRoot,
                        wristPivotPoint,
                        out wristPivotPositionInFollowRoot,
                        out wristPivotRotationInFollowRoot
                    );
                    CapturePartInPivot(
                        wristPivotPoint.position,
                        wristPivotPoint.rotation,
                        wristRoot,
                        out wristOpenPositionInPivot,
                        out wristOpenRotationInPivot
                    );

                    if (wristAxisReferencePoint != null)
                    {
                        CaptureRelativePose(
                            activeWristFollowRoot,
                            wristAxisReferencePoint,
                            out wristAxisReferencePositionInFollowRoot,
                            out wristAxisReferenceRotationInFollowRoot
                        );
                        wristAxisReferenceCaptured = true;
                    }
                }
                else
                {
                    Debug.LogError(
                        "Wrist Follow Root is invalid. Assign Cube.003 as the stable parent and Circle.002 as Wrist Root.",
                        this
                    );
                }
            }
        }

        if (shoulderPoint != null && armRoot != null)
        {
            CaptureRelativePose(
                transform.root,
                shoulderPoint,
                out shoulderPivotPositionInRobot,
                out shoulderPivotRotationInRobot
            );
            CapturePartInPivot(
                shoulderPoint.position,
                shoulderPoint.rotation,
                armRoot,
                out armOpenPositionInPivot,
                out armOpenRotationInPivot
            );
        }

        Transform secondPivot = secondShoulderPoint != null ? secondShoulderPoint : shoulderPoint;
        if (secondPivot != null && secondArmRoot != null)
        {
            CaptureRelativePose(
                transform.root,
                secondPivot,
                out secondPivotPositionInRobot,
                out secondPivotRotationInRobot
            );
            CapturePartInPivot(
                secondPivot.position,
                secondPivot.rotation,
                secondArmRoot,
                out secondArmOpenPositionInPivot,
                out secondArmOpenRotationInPivot
            );
        }

        elbowDriveRoot = armRoot != null ? armRoot : forearmRoot;
        if (elbowPoint != null && elbowDriveRoot != null)
        {
            activeElbowFollowRoot = useForearmParentAsElbowFrame && elbowDriveRoot.parent != null
                ? elbowDriveRoot.parent
                : (elbowFollowRoot != null ? elbowFollowRoot : armRoot);
            // For this model the elbow drives Cube.003. elbowPoint is its child;
            // RotateAround keeps the exact child point fixed in world space.
            elbowConfigurationValid = true;

            if (elbowConfigurationValid)
            {
                if (activeElbowFollowRoot != null)
                {
                    CaptureRelativePose(
                        activeElbowFollowRoot,
                        elbowPoint,
                        out elbowPivotPositionInFollowRoot,
                        out elbowPivotRotationInFollowRoot
                    );
                }
            }
            else
            {
                Debug.LogError(
                    "Elbow configuration is invalid. Assign Arm Root (Cube.003) and elbowPoint.",
                    this
                );
            }
        }

        OpenJaws();
    }

    void Update()
    {
        UpdateArm();

        if (autoGrabWhenJawsClosed && JawsClosed && !IsHolding)
        {
            TryGrab();
        }

        UpdateJaws();
        UpdateWrist();
    }

    void LateUpdate()
    {
        UpdateShoulder();
        UpdateElbow();
        PoseReached = Mathf.Abs(currentArmAngle - targetArmAngle) < 0.1f &&
                      Mathf.Abs(currentElbowAngle - targetElbowAngle) < 0.1f;
    }

    public static GripperController FindController(Transform owner)
    {
        if (owner == null)
        {
            return null;
        }

        GripperController[] controllers = owner.GetComponentsInChildren<GripperController>(true);
        GripperController best = null;
        int bestDistance = int.MaxValue;

        foreach (GripperController controller in controllers)
        {
            int distance = controller.DistanceToControlledJaws();
            if (distance < bestDistance)
            {
                best = controller;
                bestDistance = distance;
            }
        }

        return best;
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
        JawClosure01 = 1f;
        TryGrab();
        targetJawAngle = IsHolding ? GetBallHoldAngle() : Mathf.Max(0f, closeAngle);
    }

    float GetBallHoldAngle()
    {
        return Mathf.Clamp(ballHoldAngle, 0f, Mathf.Max(0f, closeAngle));
    }

    Vector3 ResolveJawAxis(Transform motionSpace)
    {
        if (!useVerticalJawPlane || motionSpace == null || holdPoint == null)
        {
            return rotationAxis.sqrMagnitude > 0.0001f ? rotationAxis.normalized : Vector3.forward;
        }

        Vector3 origin = shoulderPoint != null ? shoulderPoint.position : transform.root.position;
        Vector3 armForward = Vector3.ProjectOnPlane(holdPoint.position - origin, Vector3.up).normalized;
        if (armForward.sqrMagnitude < 0.0001f)
        {
            armForward = Vector3.ProjectOnPlane(transform.root.forward, Vector3.up).normalized;
        }

        Vector3 worldAxis = Vector3.Cross(armForward, Vector3.up).normalized;
        return motionSpace.InverseTransformDirection(worldAxis).normalized;
    }

    void CaptureJawBasePose()
    {
        if (jawPivotPoint == null || leftJaw == null || rightJaw == null)
        {
            return;
        }

        leftBasePositionInPivot = jawPivotPoint.InverseTransformPoint(leftJaw.position);
        rightBasePositionInPivot = jawPivotPoint.InverseTransformPoint(rightJaw.position);
        leftBaseRotationInPivot = Quaternion.Inverse(jawPivotPoint.rotation) * leftJaw.rotation;
        rightBaseRotationInPivot = Quaternion.Inverse(jawPivotPoint.rotation) * rightJaw.rotation;
    }

    public void OpenJaws()
    {
        JawsClosed = false;
        JawClosure01 = 0f;
        targetJawAngle = -Mathf.Max(0f, openAngle);
    }

    public void SetJawClosure01(float closure01, bool tryGrab = false)
    {
        JawClosure01 = Mathf.Clamp01(closure01);
        JawsClosed = JawClosure01 >= 0.5f;

        if (tryGrab && JawsClosed)
        {
            TryGrab();
        }

        if (JawClosure01 <= 0.001f)
        {
            Release();
        }

        targetJawAngle = IsHolding
            ? GetBallHoldAngle()
            : Mathf.Lerp(-Mathf.Max(0f, openAngle), Mathf.Max(0f, closeAngle), JawClosure01);
    }

    void UpdateJaws()
    {
        if (leftJaw == null || rightJaw == null)
        {
            return;
        }

        currentJawAngle = Mathf.MoveTowards(
            currentJawAngle,
            targetJawAngle,
            Mathf.Max(0f, jawDegreesPerSecond) * Time.deltaTime
        );

        float sign = invertDirection ? -1f : 1f;
        Transform motionSpace = jawPivotPoint != null ? jawPivotPoint : leftJaw.parent;
        Vector3 axis = ResolveJawAxis(motionSpace);
        Quaternion leftMotion = Quaternion.AngleAxis(currentJawAngle * sign, axis);
        Quaternion rightMotion = Quaternion.AngleAxis(-currentJawAngle * sign, axis);

        if (jawPivotPoint != null)
        {
            leftJaw.position = jawPivotPoint.TransformPoint(leftMotion * leftBasePositionInPivot);
            leftJaw.rotation = jawPivotPoint.rotation * leftMotion * leftBaseRotationInPivot;
            rightJaw.position = jawPivotPoint.TransformPoint(rightMotion * rightBasePositionInPivot);
            rightJaw.rotation = jawPivotPoint.rotation * rightMotion * rightBaseRotationInPivot;
            return;
        }

        leftJaw.localRotation = leftMotion * leftOpenRotation;
        rightJaw.localRotation = rightMotion * rightOpenRotation;
    }

    public void Close()
    {
        CloseJaws();
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
        SetPose(ArmPose.Idle);
    }

    public void SetPose(ArmPose pose)
    {
        CurrentPose = pose;
        targetArmAngle = pose switch
        {
            ArmPose.Reach => GetLowerArmLimit(),
            ArmPose.Carry => GetUpperArmLimit(),
            _ => 0f
        };
        targetElbowAngle = pose switch
        {
            ArmPose.Reach => elbowReachAngle,
            ArmPose.Carry => elbowCarryAngle,
            _ => 0f
        };
        PoseReached = false;
    }

    public void JogArm(float direction, float deltaTime)
    {
        float lowerAngle = GetLowerArmLimit();
        float upperAngle = GetUpperArmLimit();
        float speed = Mathf.Clamp(armDegreesPerSecond, 0f, 30f);

        targetArmAngle = Mathf.Clamp(
            currentArmAngle + Mathf.Clamp(direction, -1f, 1f) * speed * deltaTime,
            lowerAngle,
            upperAngle
        );

        // During manual jogging the elbow holds its angle. Otherwise it can
        // compensate the shoulder and make both keys look like "raise".
        targetElbowAngle = currentElbowAngle;
        CurrentPose = direction < 0f ? ArmPose.Reach : ArmPose.Carry;
        PoseReached = false;
    }

    float GetLowerArmLimit()
    {
        float configuredLimit = -Mathf.Abs(reachAngle) - Mathf.Abs(reachExtraDrop);
        return Mathf.Min(configuredLimit, minShoulderDownAngle);
    }

    float GetUpperArmLimit()
    {
        return Mathf.Abs(carryAngle);
    }

    public void StopArm()
    {
        targetArmAngle = currentArmAngle;
        targetElbowAngle = currentElbowAngle;
        PoseReached = true;
    }

    /// <summary>
    /// Bends only the part after elbowPoint: Forearm Root, rotatePoint and claws.
    /// It does not change the shoulder target.
    /// </summary>
    public void JogElbow(float direction, float deltaTime)
    {
        if (!elbowConfigurationValid)
        {
            if (!warnedInvalidElbow)
            {
                Debug.LogWarning("Elbow is not configured. Check Forearm Root and elbowPoint in Gripper Controller.", this);
                warnedInvalidElbow = true;
            }

            return;
        }

        float min = Mathf.Min(elbowMinAngle, elbowMaxAngle);
        float max = Mathf.Max(elbowMinAngle, elbowMaxAngle);
        float speed = Mathf.Clamp(elbowDegreesPerSecond, 0f, 45f);

        targetElbowAngle = Mathf.Clamp(
            currentElbowAngle + Mathf.Clamp(direction, -1f, 1f) * speed * deltaTime,
            min,
            max
        );
        PoseReached = false;
    }

    public void StopElbow()
    {
        targetElbowAngle = currentElbowAngle;
        PoseReached = true;
    }

    public void SetWristRoll(float roll)
    {
        SetWristAngleDegrees(Mathf.Clamp01(roll) * Mathf.Abs(wristRollMaxAngle));
    }

    public void SetWristAngleDegrees(float degrees)
    {
        if (wristRoot == null)
        {
            return;
        }

        float direction = clockwiseWristRotation ? -1f : 1f;
        targetWristAngle = Mathf.Clamp(degrees, 0f, Mathf.Abs(wristRollMaxAngle)) * direction;
        Vector3 axis = ResolveWristAxis();
        wristTargetRotation = wristOpenRotation * Quaternion.AngleAxis(
            targetWristAngle + wristAlignmentOffsetDegrees,
            axis
        );
    }

    Vector3 ResolveWristAxis()
    {
        if (usePerpendicularWristPlane)
        {
            return Vector3.right;
        }

        return wristRollAxis.sqrMagnitude > 0.0001f
            ? wristRollAxis.normalized
            : Vector3.right;
    }

    Vector3 ResolveWristWorldAxis(Vector3 wristPivotPosition, Quaternion wristPivotRotation)
    {
        if (alignWristAxisWithArm)
        {
            Vector3 referencePosition = Vector3.zero;
            bool hasReference = false;

            if (wristAxisReferenceCaptured && activeWristFollowRoot != null)
            {
                GetWorldPose(
                    activeWristFollowRoot,
                    wristAxisReferencePositionInFollowRoot,
                    wristAxisReferenceRotationInFollowRoot,
                    out referencePosition,
                    out _
                );
                hasReference = true;
            }
            else if (elbowConfigurationValid && activeElbowFollowRoot != null)
            {
                GetWorldPose(
                    activeElbowFollowRoot,
                    elbowPivotPositionInFollowRoot,
                    elbowPivotRotationInFollowRoot,
                    out referencePosition,
                    out _
                );
                hasReference = true;
            }
            if (hasReference)
            {
                Vector3 armAxis = wristPivotPosition - referencePosition;
                if (armAxis.sqrMagnitude > 0.000001f)
                {
                    return armAxis.normalized;
                }
            }
        }

        return wristPivotRotation * ResolveWristAxis();
    }

    void UpdateWrist()
    {
        if (wristRoot == null)
        {
            return;
        }

        currentWristAngle = Mathf.MoveTowards(
            currentWristAngle,
            targetWristAngle,
            Mathf.Max(0f, wristDegreesPerSecond) * Time.deltaTime
        );

        if (wristPivotPoint != null && wristConfigurationValid)
        {
            float calibratedWristAngle = currentWristAngle + wristAlignmentOffsetDegrees;
            GetWorldPose(
                activeWristFollowRoot,
                wristPivotPositionInFollowRoot,
                wristPivotRotationInFollowRoot,
                out Vector3 wristPivotPosition,
                out Quaternion wristPivotRotation
            );
            Vector3 worldAxis = ResolveWristWorldAxis(wristPivotPosition, wristPivotRotation);

            if (wristUsesForearmFrame)
            {
                // UpdateArm has already restored the forearm pose this frame.
                // Apply only the wrist rotation and keep rotatePoint fixed in world space.
                Quaternion wristMotion = Quaternion.AngleAxis(
                    calibratedWristAngle,
                    worldAxis
                );
                wristRoot.position = wristPivotPosition +
                                     wristMotion * (wristRoot.position - wristPivotPosition);
                wristRoot.rotation = wristMotion * wristRoot.rotation;
                return;
            }

            Vector3 axisInPivot = Quaternion.Inverse(wristPivotRotation) * worldAxis;

            ApplyPartInPivot(
                wristPivotPosition,
                wristPivotRotation,
                wristRoot,
                wristOpenPositionInPivot,
                wristOpenRotationInPivot,
                axisInPivot,
                calibratedWristAngle
            );
            return;
        }

        wristRoot.localRotation = Quaternion.RotateTowards(
            wristRoot.localRotation,
            wristTargetRotation,
            Mathf.Max(0f, wristDegreesPerSecond) * Time.deltaTime
        );
    }

    void UpdateArm()
    {
        if ((shoulderPoint == null || armRoot == null) && !elbowConfigurationValid)
        {
            PoseReached = true;
        }
    }

    // The shoulder and elbow share Cube.003. Applying small world-space deltas
    // lets both joints stay at arbitrary angles instead of one resetting the other.
    void UpdateShoulder()
    {
        if (shoulderPoint == null || armRoot == null)
        {
            return;
        }

        float nextAngle = Mathf.MoveTowards(
            currentArmAngle,
            targetArmAngle,
            Mathf.Clamp(armDegreesPerSecond, 0f, 30f) * Time.deltaTime
        );
        float deltaAngle = nextAngle - currentArmAngle;
        if (Mathf.Abs(deltaAngle) < 0.00001f)
        {
            return;
        }

        // shoulderPoint is a child of Cube.003, so its live transform moves when
        // the elbow turns. Use the captured shoulder motor instead; this is the
        // same fixed centre and axis used by the original working shoulder code.
        GetWorldPose(
            transform.root,
            shoulderPivotPositionInRobot,
            shoulderPivotRotationInRobot,
            out Vector3 shoulderPosition,
            out Quaternion shoulderRotation
        );
        Vector3 axis = shoulderRotation *
                       (shoulderAxis.sqrMagnitude > 0.0001f ? shoulderAxis.normalized : Vector3.forward);
        armRoot.RotateAround(shoulderPosition, axis.normalized, deltaAngle);

        Transform secondPivot = secondShoulderPoint != null ? secondShoulderPoint : shoulderPoint;
        if (secondArmRoot != null && secondPivot != null && !secondArmRoot.IsChildOf(armRoot))
        {
            float secondDelta = invertSecondShoulder ? -deltaAngle : deltaAngle;
            GetWorldPose(
                transform.root,
                secondPivotPositionInRobot,
                secondPivotRotationInRobot,
                out Vector3 secondPosition,
                out Quaternion secondRotation
            );
            Vector3 secondAxis = secondRotation *
                                 (shoulderAxis.sqrMagnitude > 0.0001f ? shoulderAxis.normalized : Vector3.forward);
            secondArmRoot.RotateAround(secondPosition, secondAxis.normalized, secondDelta);
        }

        currentArmAngle = nextAngle;
    }

    void UpdateElbow()
    {
        if (!elbowConfigurationValid || elbowPoint == null || elbowDriveRoot == null)
        {
            return;
        }

        float nextAngle = Mathf.MoveTowards(
            currentElbowAngle,
            targetElbowAngle,
            Mathf.Clamp(elbowDegreesPerSecond, 0f, 45f) * Time.deltaTime
        );
        float deltaAngle = nextAngle - currentElbowAngle;
        if (Mathf.Abs(deltaAngle) < 0.00001f)
        {
            return;
        }

        Vector3 axis = useElbowLocalRightAxis
            ? elbowPoint.right
            : elbowPoint.TransformDirection(elbowAxis.sqrMagnitude > 0.0001f ? elbowAxis.normalized : Vector3.right);

        elbowDriveRoot.RotateAround(elbowPoint.position, axis.normalized, deltaAngle);
        currentElbowAngle = nextAngle;
    }

    static void CaptureRelativePose(
        Transform frame,
        Transform item,
        out Vector3 positionInFrame,
        out Quaternion rotationInFrame
    )
    {
        positionInFrame = frame.InverseTransformPoint(item.position);
        rotationInFrame = Quaternion.Inverse(frame.rotation) * item.rotation;
    }

    static void CapturePartInPivot(
        Vector3 pivotPosition,
        Quaternion pivotRotation,
        Transform part,
        out Vector3 positionInPivot,
        out Quaternion rotationInPivot
    )
    {
        positionInPivot = Quaternion.Inverse(pivotRotation) * (part.position - pivotPosition);
        rotationInPivot = Quaternion.Inverse(pivotRotation) * part.rotation;
    }

    static void GetWorldPose(
        Transform frame,
        Vector3 positionInFrame,
        Quaternion rotationInFrame,
        out Vector3 worldPosition,
        out Quaternion worldRotation
    )
    {
        worldPosition = frame.TransformPoint(positionInFrame);
        worldRotation = frame.rotation * rotationInFrame;
    }

    static void ApplyPartInPivot(
        Vector3 pivotPosition,
        Quaternion pivotRotation,
        Transform part,
        Vector3 basePositionInPivot,
        Quaternion baseRotationInPivot,
        Vector3 axis,
        float angle
    )
    {
        Quaternion motion = Quaternion.AngleAxis(angle, axis);
        part.position = pivotPosition + pivotRotation * (motion * basePositionInPivot);
        part.rotation = pivotRotation * motion * baseRotationInPivot;
    }

    int DistanceToControlledJaws()
    {
        int leftDistance = AncestorDistance(leftJaw, transform);
        int rightDistance = AncestorDistance(rightJaw, transform);

        if (leftDistance == int.MaxValue || rightDistance == int.MaxValue)
        {
            return 100000 + TransformDepth(transform);
        }

        return leftDistance + rightDistance;
    }

    static int AncestorDistance(Transform child, Transform ancestor)
    {
        if (child == null || ancestor == null)
        {
            return int.MaxValue;
        }

        int distance = 0;
        for (Transform current = child; current != null; current = current.parent)
        {
            if (current == ancestor)
            {
                return distance;
            }

            distance++;
        }

        return int.MaxValue;
    }

    static int TransformDepth(Transform item)
    {
        int depth = 0;
        for (Transform current = item; current != null; current = current.parent)
        {
            depth++;
        }

        return depth;
    }

    public void Release()
    {
        if (!IsHolding)
        {
            return;
        }

        heldBall.SetParent(originalParent, true);

        if (heldColliders != null)
        {
            for (int i = 0; i < heldColliders.Length; i++)
            {
                if (heldColliders[i] != null)
                {
                    heldColliders[i].enabled = heldColliderStates[i];
                }
            }
        }

        if (heldRb != null)
        {
            heldRb.isKinematic = heldRbWasKinematic;
            heldRb.useGravity = heldRbUsedGravity;
            heldRb.linearVelocity = Vector3.zero;
            heldRb.angularVelocity = Vector3.zero;
        }

        heldBall = null;
        heldRb = null;
        heldColliders = null;
        heldColliderStates = null;
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

        if (targetBall != null && IsBallInGrabRange(targetBall))
        {
            GrabBall(targetBall);
            return;
        }

        Collider[] hits = Physics.OverlapSphere(
            holdPoint.position,
            grabRadius,
            Physics.AllLayers,
            QueryTriggerInteraction.Collide
        );
        foreach (Collider hit in hits)
        {
            Transform candidate = hit.attachedRigidbody != null
                ? hit.attachedRigidbody.transform
                : hit.transform;

            if (candidate.IsChildOf(transform.root) || !HasTargetBallTag(candidate))
            {
                continue;
            }

            targetBall = candidate;
            GrabBall(candidate);
            return;
        }
    }

    Transform FindSceneTargetBall()
    {
        try
        {
            GameObject[] candidates = GameObject.FindGameObjectsWithTag(targetBallTag);
            foreach (GameObject candidate in candidates)
            {
                if (candidate.transform.IsChildOf(transform.root))
                {
                    continue;
                }

                if (candidate.GetComponentInChildren<Collider>() != null &&
                    candidate.GetComponent<Rigidbody>() != null)
                {
                    return candidate.transform;
                }
            }
        }
        catch (UnityException)
        {
            Debug.LogError($"Tag '{targetBallTag}' is not defined. Add it in Tags and Layers.", this);
        }

        return null;
    }

    bool HasTargetBallTag(Transform candidate)
    {
        for (Transform current = candidate; current != null; current = current.parent)
        {
            if (current.gameObject.tag == targetBallTag)
            {
                return true;
            }
        }

        return false;
    }

    bool IsBallInGrabRange(Transform ball)
    {
        if (ball == null || ball.IsChildOf(transform.root))
        {
            return false;
        }

        float radiusSquared = grabRadius * grabRadius;
        Collider[] colliders = ball.GetComponentsInChildren<Collider>();
        foreach (Collider ballCollider in colliders)
        {
            if (!ballCollider.enabled)
            {
                continue;
            }

            Vector3 closestPoint = ballCollider.ClosestPoint(holdPoint.position);
            if ((closestPoint - holdPoint.position).sqrMagnitude <= radiusSquared)
            {
                return true;
            }
        }

        return (ball.position - holdPoint.position).sqrMagnitude <= radiusSquared;
    }

    void GrabBall(Transform ball)
    {
        if (ball == null || IsHolding)
        {
            return;
        }

        heldBall = ball;
        targetBall = ball;
        originalParent = heldBall.parent;
        Vector3 ballWorldScale = heldBall.lossyScale;
        heldRb = heldBall.GetComponent<Rigidbody>();
        heldColliders = heldBall.GetComponentsInChildren<Collider>();
        heldColliderStates = new bool[heldColliders.Length];

        for (int i = 0; i < heldColliders.Length; i++)
        {
            heldColliderStates[i] = heldColliders[i].enabled;
            heldColliders[i].enabled = false;
        }

        if (heldRb != null)
        {
            heldRbWasKinematic = heldRb.isKinematic;
            heldRbUsedGravity = heldRb.useGravity;
            heldRb.isKinematic = true;
            heldRb.useGravity = false;
            heldRb.linearVelocity = Vector3.zero;
            heldRb.angularVelocity = Vector3.zero;
        }

        // Keep the world scale. Imported robot links use a scale close to 0.01,
        // so SetParent(..., false) would make the ball almost disappear.
        heldBall.SetParent(holdPoint, true);
        if (snapBallToHoldPoint)
        {
            heldBall.position = holdPoint.position;
            heldBall.rotation = holdPoint.rotation;
            SetWorldScale(heldBall, ballWorldScale);
        }


        targetJawAngle = GetBallHoldAngle();
        Debug.Log($"Gripper grabbed {heldBall.name} at Hold Point.", this);
    }

    static void SetWorldScale(Transform item, Vector3 worldScale)
    {
        Transform parent = item.parent;
        if (parent == null)
        {
            item.localScale = worldScale;
            return;
        }

        Vector3 parentScale = parent.lossyScale;
        item.localScale = new Vector3(
            SafeScaleDivision(worldScale.x, parentScale.x),
            SafeScaleDivision(worldScale.y, parentScale.y),
            SafeScaleDivision(worldScale.z, parentScale.z)
        );
    }

    static float SafeScaleDivision(float value, float divisor)
    {
        return Mathf.Abs(divisor) > 0.000001f ? value / divisor : value;
    }

    void OnDrawGizmosSelected()
    {
        if (holdPoint != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(holdPoint.position, grabRadius);
        }

        Transform axisReference = wristAxisReferencePoint != null
            ? wristAxisReferencePoint
            : elbowPoint;
        if (wristPivotPoint != null && axisReference != null)
        {
            Vector3 axis = wristPivotPoint.position - axisReference.position;
            if (axis.sqrMagnitude > 0.000001f)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(axisReference.position, wristPivotPoint.position);
                Gizmos.DrawRay(wristPivotPoint.position, axis.normalized * 0.25f);
            }
        }

    }
}
