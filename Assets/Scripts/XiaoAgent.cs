using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(TrackController))]
[RequireComponent(typeof(VirtualSensors))]
public class XiaoAgent : Agent
{
    public Transform targetBall;
    public Transform targetZone;
    public float arenaRadius = 5f;
    public int episodeStepLimit = 2500;

    TrackController tracks;
    VirtualSensors sensors;
    GripperController gripper;
    Vector3 startPosition;
    Quaternion startRotation;
    Vector3 ballStartPosition;
    float previousTargetDistance;

    void Awake()
    {
        tracks = GetComponent<TrackController>();
        sensors = GetComponent<VirtualSensors>();
        gripper = GripperController.FindController(transform);
    }

    public override void Initialize()
    {
        startPosition = transform.position;
        startRotation = transform.rotation;
        if (targetBall != null)
        {
            ballStartPosition = targetBall.position;
        }
    }

    public override void OnEpisodeBegin()
    {
        tracks.Stop();
        transform.SetPositionAndRotation(startPosition, startRotation);

        if (gripper != null)
        {
            gripper.Release();
        }

        if (targetBall != null)
        {
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

            Vector2 random = Random.insideUnitCircle * 1.5f;
            targetBall.position = ballStartPosition + new Vector3(random.x, 0f, random.y);
        }

        previousTargetDistance = DistanceToCurrentGoal();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localGoal = transform.InverseTransformPoint(CurrentGoalPosition());
        Vector2 pwm = tracks.CurrentPwm / 100f;

        sensor.AddObservation(Mathf.Clamp(localGoal.x / arenaRadius, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(localGoal.z / arenaRadius, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp01(localGoal.magnitude / arenaRadius));
        sensor.AddObservation(sensors.Ultrasonic01);
        sensor.AddObservation(sensors.LeftIR);
        sensor.AddObservation(sensors.RightIR);
        sensor.AddObservation(sensors.GripperIR);
        sensor.AddObservation(gripper != null && gripper.IsHolding ? 1f : 0f);
        sensor.AddObservation(pwm.x);
        sensor.AddObservation(pwm.y);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gas = actions.ContinuousActions[0];
        float steer = actions.ContinuousActions[1];
        float grab = actions.ContinuousActions.Length > 2 ? actions.ContinuousActions[2] : 0f;

        tracks.SetCommand(gas, steer);
        if (gripper != null)
        {
            gripper.SetGripper(grab);
        }

        float currentDistance = DistanceToCurrentGoal();
        AddReward((previousTargetDistance - currentDistance) * 0.02f);
        AddReward(-0.0005f);
        previousTargetDistance = currentDistance;

        if (sensors.LeftIR > 0.5f || sensors.RightIR > 0.5f)
        {
            AddReward(-0.002f);
        }

        if (gripper != null && gripper.IsHolding)
        {
            AddReward(0.002f);
        }

        if (targetZone != null && gripper != null && gripper.IsHolding &&
            Vector3.Distance(transform.position, targetZone.position) < 0.6f)
        {
            AddReward(2.0f);
            EndEpisode();
        }

        if (Vector3.Distance(startPosition, transform.position) > arenaRadius || StepCount >= episodeStepLimit)
        {
            AddReward(-0.25f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        float gas = 0f;
        float steer = 0f;
        float grab = 0f;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) gas += 1f;
            if (keyboard.sKey.isPressed) gas -= 1f;
            if (keyboard.aKey.isPressed) steer -= 1f;
            if (keyboard.dKey.isPressed) steer += 1f;
            if (keyboard.spaceKey.isPressed) grab = 1f;
        }
#elif ENABLE_LEGACY_INPUT_MANAGER
        gas = Input.GetAxis("Vertical");
        steer = Input.GetAxis("Horizontal");
        grab = Input.GetKey(KeyCode.Space) ? 1f : 0f;
#endif

        actions[0] = gas;
        actions[1] = steer;
        actions[2] = grab;
    }

    Vector3 CurrentGoalPosition()
    {
        if (gripper != null && gripper.IsHolding && targetZone != null)
        {
            return targetZone.position;
        }

        return targetBall != null ? targetBall.position : startPosition;
    }

    float DistanceToCurrentGoal()
    {
        return Vector3.Distance(transform.position, CurrentGoalPosition());
    }
}
