using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(TrackController))]
[RequireComponent(typeof(VirtualSensors))]
public class RobotBrain : Agent
{
    public enum TrainingGoal
    {
        ApproachBall,
        GrabBall
    }

    [Header("Task")]
    public Transform targetBall;
    public Transform targetZone;
    public Transform approachPoint;
    public TrainingGoal trainingGoal = TrainingGoal.ApproachBall;
    public float arenaRadius = 5f;
    public int episodeStepLimit = 1200;
    public float approachSuccessDistance = 0.45f;
    public bool randomizeBallOnReset = false;

    [Header("Rewards")]
    public float approachTerminalReward = 3.0f;
    public float grabTerminalReward = 5.0f;
    public float distanceRewardScale = 0.7f;
    public float timePenalty = 0.0001f;
    public float actionRatePenalty = 0.002f;
    public float obstaclePenalty = 0.01f;

    [Header("Perception")]
    public SimulatedYoloCamera yoloCamera;

    [Header("Camera servo")]
    public Transform cameraPivot;
    public float cameraTurnSpeed = 90f;
    public float cameraMaxAngle = 60f;

    Rigidbody rb;
    TrackController tracks;
    VirtualSensors sensors;
    GripperController gripper;

    Vector3 startPosition;
    Quaternion startRotation;
    Vector3 ballStartPosition;
    Quaternion cameraStartRotation;

    float cameraServoAngle;
    float lastKnownBallDirection;
    float timeSinceLastDetection;
    float previousBallDistance;
    float previousGas;
    float previousSteer;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        tracks = GetComponent<TrackController>();
        sensors = GetComponent<VirtualSensors>();
        gripper = GetComponentInChildren<GripperController>();
    }

    public override void Initialize()
    {
        startPosition = transform.position;
        startRotation = transform.rotation;

        if (targetBall != null)
        {
            ballStartPosition = targetBall.position;
        }

        if (cameraPivot != null)
        {
            cameraStartRotation = cameraPivot.localRotation;
        }
    }

    public override void OnEpisodeBegin()
    {
        tracks.Stop();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.SetPositionAndRotation(startPosition, startRotation);

        cameraServoAngle = 0f;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = cameraStartRotation;
        }

        if (gripper != null)
        {
            gripper.Release();
        }

        ResetBall();
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousBallDistance = DistanceToBall();
        previousGas = 0f;
        previousSteer = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (yoloCamera != null)
        {
            yoloCamera.UpdateDetection();
            if (yoloCamera.IsTargetVisible)
            {
                lastKnownBallDirection = yoloCamera.TargetX;
                timeSinceLastDetection = 0f;
            }
            else
            {
                timeSinceLastDetection += Time.deltaTime;
            }
        }

        bool visible = yoloCamera != null && yoloCamera.IsTargetVisible;
        bool hasBall = gripper != null && gripper.IsHolding;
        Vector3 offsetFromStart = transform.position - startPosition;
        float heading = Mathf.DeltaAngle(0f, transform.eulerAngles.y) / 180f;
        float speed01 = Mathf.Clamp01(rb.linearVelocity.magnitude / 2f);
        float cameraServo01 = cameraMaxAngle > 0f ? Mathf.Clamp(cameraServoAngle / cameraMaxAngle, -1f, 1f) : 0f;

        // Exactly 15 observations, matching Practice 3 order.
        sensor.AddObservation(sensors.Ultrasonic01);
        sensor.AddObservation(sensors.LeftIR);
        sensor.AddObservation(sensors.RightIR);
        sensor.AddObservation(sensors.GripperIR);
        sensor.AddObservation(visible ? yoloCamera.TargetX : 0f);
        sensor.AddObservation(visible ? yoloCamera.Distance01 : 1f);
        sensor.AddObservation(lastKnownBallDirection);
        sensor.AddObservation(visible ? 1f : 0f);
        sensor.AddObservation(cameraServo01);
        sensor.AddObservation(hasBall ? 1f : 0f);
        sensor.AddObservation(Mathf.Clamp(offsetFromStart.x / arenaRadius, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(offsetFromStart.z / arenaRadius, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(heading, -1f, 1f));
        sensor.AddObservation(speed01);
        sensor.AddObservation(Mathf.Clamp01(timeSinceLastDetection / 5f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float cameraTurn = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        tracks.SetCommand(gas, steer);
        ApplyCameraServo(cameraTurn);

        int gripperAction = actions.DiscreteActions.Length > 0 ? actions.DiscreteActions[0] : 0;
        ApplyGripperAction(gripperAction);
        ApplyRewards(gas, steer);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions;
        ActionSegment<int> discrete = actionsOut.DiscreteActions;

        float gas = 0f;
        float steer = 0f;
        float cameraTurn = 0f;
        int gripperAction = 0;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) gas += 1f;
            if (keyboard.sKey.isPressed) gas -= 1f;
            if (keyboard.aKey.isPressed) steer -= 1f;
            if (keyboard.dKey.isPressed) steer += 1f;
            if (keyboard.qKey.isPressed) cameraTurn -= 1f;
            if (keyboard.eKey.isPressed) cameraTurn += 1f;
            if (keyboard.spaceKey.isPressed) gripperAction = 1;
            if (keyboard.leftShiftKey.isPressed) gripperAction = 2;
        }
#elif ENABLE_LEGACY_INPUT_MANAGER
        gas = Input.GetAxis("Vertical");
        steer = Input.GetAxis("Horizontal");
        if (Input.GetKey(KeyCode.Q)) cameraTurn -= 1f;
        if (Input.GetKey(KeyCode.E)) cameraTurn += 1f;
        if (Input.GetKey(KeyCode.Space)) gripperAction = 1;
        if (Input.GetKey(KeyCode.LeftShift)) gripperAction = 2;
#endif

        if (continuous.Length >= 3)
        {
            continuous[0] = gas;
            continuous[1] = steer;
            continuous[2] = cameraTurn;
        }

        if (discrete.Length > 0)
        {
            discrete[0] = gripperAction;
        }
    }

    void ApplyCameraServo(float cameraTurn)
    {
        cameraServoAngle = Mathf.Clamp(
            cameraServoAngle + cameraTurn * cameraTurnSpeed * Time.fixedDeltaTime,
            -cameraMaxAngle,
            cameraMaxAngle
        );

        if (cameraPivot != null)
        {
            cameraPivot.localRotation = cameraStartRotation * Quaternion.Euler(0f, cameraServoAngle, 0f);
        }
    }

    void ApplyGripperAction(int action)
    {
        if (gripper == null)
        {
            return;
        }

        if (action == 1)
        {
            gripper.SetGripper(1f);
        }
        else if (action == 2)
        {
            gripper.SetGripper(-1f);
        }
    }

    void ApplyRewards(float gas, float steer)
    {
        bool hasBall = gripper != null && gripper.IsHolding;
        bool visible = yoloCamera != null && yoloCamera.IsTargetVisible;
        float currentBallDistance = DistanceToBall();
        float distanceDelta = previousBallDistance - currentBallDistance;

        AddReward(distanceDelta * distanceRewardScale);
        AddReward(-timePenalty);

        float actionRate = Mathf.Abs(gas - previousGas) + Mathf.Abs(steer - previousSteer);
        AddReward(-actionRatePenalty * actionRate);

        if (visible)
        {
            AddReward((1f - Mathf.Abs(yoloCamera.TargetX)) * 0.002f);
        }

        if (sensors.Ultrasonic01 < 0.15f || sensors.LeftIR > 0.5f || sensors.RightIR > 0.5f)
        {
            AddReward(-obstaclePenalty);
        }

        previousBallDistance = currentBallDistance;
        previousGas = gas;
        previousSteer = steer;

        if (trainingGoal == TrainingGoal.ApproachBall && currentBallDistance <= approachSuccessDistance)
        {
            AddReward(approachTerminalReward);
            EndEpisode();
            return;
        }

        if (trainingGoal == TrainingGoal.GrabBall && hasBall)
        {
            AddReward(grabTerminalReward);
            EndEpisode();
            return;
        }

        if (trainingGoal == TrainingGoal.GrabBall && currentBallDistance <= approachSuccessDistance)
        {
            AddReward(0.002f);
        }

        if (Vector3.Distance(startPosition, transform.position) > arenaRadius)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        if (StepCount >= episodeStepLimit)
        {
            AddReward(-0.1f);
            EndEpisode();
        }
    }

    void ResetBall()
    {
        if (targetBall == null)
        {
            return;
        }

        Rigidbody ballRb = targetBall.GetComponent<Rigidbody>();
        Collider ballCollider = targetBall.GetComponent<Collider>();

        if (ballCollider != null)
        {
            ballCollider.enabled = true;
        }

        if (ballRb != null)
        {
            ballRb.isKinematic = false;
            ballRb.linearVelocity = Vector3.zero;
            ballRb.angularVelocity = Vector3.zero;
        }

        if (randomizeBallOnReset)
        {
            Vector2 random = Random.insideUnitCircle * 1.5f;
            targetBall.position = ballStartPosition + new Vector3(random.x, 0f, random.y);
        }
        else
        {
            targetBall.position = ballStartPosition;
        }
    }

    float DistanceToBall()
    {
        if (targetBall == null)
        {
            return arenaRadius;
        }

        Vector3 referencePosition = approachPoint != null ? approachPoint.position : transform.position;
        return Vector3.Distance(referencePosition, targetBall.position);
    }
}
