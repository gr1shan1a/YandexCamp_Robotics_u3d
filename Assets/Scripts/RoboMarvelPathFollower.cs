using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(RoboMarvelRouteReceiver))]
[RequireComponent(typeof(GripperIRAutoClose))]
public sealed class RoboMarvelPathFollower : MonoBehaviour
{
    [Header("References")]
    public RoboMarvelRouteReceiver routeReceiver;
    public ROSBridge rosBridge;
    public TrackController tracks;
    public VirtualSensors sensors;
    public RobotBrain robotBrain;
    public GripperIRAutoClose gripperIrAutoClose;

    [Header("Control")]
    [Tooltip("Physical route driving is blocked until this is explicitly enabled.")]
    public bool autonomousRouteEnabled;
    public bool followNewRoutesAutomatically = true;
    [Min(1f)] public float commandRateHz = 20f;
    [Range(0.05f, 1f)] public float cruiseCommand = 0.45f;
    [Range(0.02f, 1f)] public float minimumDriveCommand = 0.18f;
    [Min(0.05f)] public float steeringGain = 1.5f;
    [Range(0.05f, 1f)] public float maximumSteeringCommand = 0.75f;
    [Range(1f, 90f)] public float turnInPlaceAngleDegrees = 28f;
    public bool invertSteering;

    [Header("Waypoints")]
    [Min(0.02f)] public float waypointTolerance = 0.20f;
    [Min(0.02f)] public float finalGoalTolerance = 0.35f;
    [Min(0.05f)] public float lookAheadDistance = 0.35f;

    [Header("Safety")]
    public bool stopWhenRouteStreamIsStale = true;
    [Min(0.5f)] public float routeStreamTimeout = 3f;
    public bool stopForObstacleSensors = true;
    [Range(0f, 1f)]
    [Tooltip("Normalized ultrasonic distance. With a 2 m range, 0.12 is 24 cm.")]
    public float ultrasonicStop01 = 0.12f;
    public bool stopForFrontIr = true;

    [Header("Control handoff")]
    [Tooltip("Disable the ML agent while the route follower owns /cmd_vel.")]
    public bool disableRobotBrainWhileFollowing = true;
    [Tooltip("At the final waypoint, re-enable RobotBrain for YOLO final approach and grabbing.")]
    public bool handoffToRobotBrainAtGoal = true;

    [Header("Diagnostics (read only)")]
    [SerializeField] bool routeActive;
    [SerializeField] string state = "idle";
    [SerializeField] int routeSequence = -1;
    [SerializeField] int waypointIndex;
    [SerializeField] int waypointCount;
    [SerializeField] float distanceToWaypoint;
    [SerializeField] float headingErrorDegrees;
    [SerializeField] float linearCommand;
    [SerializeField] float steeringCommand;

    float nextCommandTime;
    bool brainWasEnabled;
    bool brainMovementWasEnabled;
    bool manualTeleopWasEnabled;
    bool ownsControl;

    public bool RouteActive => routeActive;
    public string State => state;

    void Awake()
    {
        if (routeReceiver == null)
        {
            routeReceiver = GetComponent<RoboMarvelRouteReceiver>();
        }

        if (rosBridge == null)
        {
            rosBridge = GetComponent<ROSBridge>();
        }

        if (tracks == null)
        {
            tracks = GetComponent<TrackController>();
        }

        if (sensors == null)
        {
            sensors = GetComponent<VirtualSensors>();
        }

        if (robotBrain == null)
        {
            robotBrain = GetComponent<RobotBrain>();
        }

        if (gripperIrAutoClose == null)
        {
            gripperIrAutoClose = GetComponent<GripperIRAutoClose>();
        }
    }

    void OnEnable()
    {
        if (routeReceiver != null)
        {
            routeReceiver.RouteUpdated += OnRouteUpdated;
        }
    }

    void Start()
    {
        if (followNewRoutesAutomatically &&
            routeReceiver != null &&
            routeReceiver.HasRoute)
        {
            BeginFollowing();
        }
    }

    void OnDisable()
    {
        if (routeReceiver != null)
        {
            routeReceiver.RouteUpdated -= OnRouteUpdated;
        }

        StopDrive();
        RestoreManualControl();
    }

    void Update()
    {
        if (!routeActive)
        {
            if (autonomousRouteEnabled &&
                followNewRoutesAutomatically &&
                routeReceiver != null &&
                routeReceiver.HasRoute &&
                routeSequence != routeReceiver.LastSequence)
            {
                BeginFollowing();
            }
            return;
        }

        if (!autonomousRouteEnabled)
        {
            CancelRoute();
            routeSequence = -1;
            state = "route drive disarmed";
            return;
        }

        if (Time.unscaledTime < nextCommandTime)
        {
            return;
        }

        nextCommandTime = Time.unscaledTime + 1f / Mathf.Max(1f, commandRateHz);
        FollowRouteStep();
    }

    void OnRouteUpdated()
    {
        if (followNewRoutesAutomatically && autonomousRouteEnabled)
        {
            BeginFollowing();
        }
        else
        {
            state = "route received; drive disarmed";
        }
    }

    [ContextMenu("Begin following current route")]
    public void BeginFollowing()
    {
        if (!autonomousRouteEnabled)
        {
            state = "route received; drive disarmed";
            StopDrive();
            return;
        }

        if (routeReceiver == null || !routeReceiver.HasRoute)
        {
            state = "waiting for route";
            return;
        }

        routeSequence = routeReceiver.LastSequence;
        waypointCount = routeReceiver.WorldWaypoints.Count;
        waypointIndex = waypointCount > 1 ? 1 : 0;
        routeActive = waypointCount >= 2;
        state = routeActive ? "following" : "route too short";
        nextCommandTime = 0f;

        if (!routeActive)
        {
            return;
        }

        if (!ownsControl)
        {
            ownsControl = true;
            if (rosBridge != null)
            {
                manualTeleopWasEnabled = rosBridge.manualTeleopEnabled;
            }

            if (robotBrain != null)
            {
                brainWasEnabled = robotBrain.enabled;
                brainMovementWasEnabled = robotBrain.isMovementEnabled;
            }
        }

        if (rosBridge != null)
        {
            rosBridge.manualTeleopEnabled = false;
            rosBridge.PublishStop();
        }

        if (robotBrain != null && disableRobotBrainWhileFollowing)
        {
            robotBrain.isMovementEnabled = false;
            robotBrain.enabled = false;
        }

        Debug.Log(
            $"[RoboMarvelPathFollower] Route {routeSequence} started with " +
            $"{waypointCount} waypoints.",
            this
        );
    }

    [ContextMenu("Stop route")]
    public void CancelRoute()
    {
        routeActive = false;
        state = "cancelled";
        StopDrive();
        RestoreManualControl();
    }

    void FollowRouteStep()
    {
        IReadOnlyList<Vector3> route = routeReceiver.WorldWaypoints;
        if (route == null || route.Count < 2 || waypointIndex >= route.Count)
        {
            CompleteRoute();
            return;
        }

        if (stopWhenRouteStreamIsStale &&
            routeReceiver.LastPacketAge > routeStreamTimeout)
        {
            state = "route stream stale";
            StopDrive();
            return;
        }

        if ((gripperIrAutoClose != null && gripperIrAutoClose.GrabLatched) ||
            FreshGripperIrDetected())
        {
            routeActive = false;
            state = "target in gripper IR";
            StopDrive();
            return;
        }

        if (ObstacleDetected())
        {
            state = "blocked by obstacle sensor";
            StopDrive();
            return;
        }

        Vector3 position = transform.position;
        position.y = 0f;

        AdvanceReachedWaypoints(route, position);
        if (waypointIndex >= route.Count)
        {
            CompleteRoute();
            return;
        }

        int targetIndex = SelectLookAheadWaypoint(route, position);
        Vector3 target = route[targetIndex];
        target.y = 0f;
        Vector3 toTarget = target - position;
        distanceToWaypoint = toTarget.magnitude;

        Vector3 driveForward = ResolveDriveForward();
        headingErrorDegrees = Vector3.SignedAngle(driveForward, toTarget.normalized, Vector3.up);
        float headingErrorRadians = headingErrorDegrees * Mathf.Deg2Rad;

        float desiredAngularSpeed = headingErrorRadians * steeringGain;
        float angularScale =
            rosBridge != null ? Mathf.Max(0.01f, rosBridge.maxAngularSpeed) : 1f;
        steeringCommand = Mathf.Clamp(
            desiredAngularSpeed / angularScale,
            -maximumSteeringCommand,
            maximumSteeringCommand
        );

        if (invertSteering)
        {
            steeringCommand = -steeringCommand;
        }

        if (Mathf.Abs(headingErrorDegrees) >= turnInPlaceAngleDegrees)
        {
            linearCommand = 0f;
            state = "turning to route";
        }
        else
        {
            float alignment = Mathf.Clamp01(
                1f - Mathf.Abs(headingErrorDegrees) / turnInPlaceAngleDegrees
            );
            float goalSlowdown = Mathf.Clamp01(
                distanceToWaypoint / Mathf.Max(waypointTolerance * 2f, 0.25f)
            );
            linearCommand = Mathf.Lerp(
                minimumDriveCommand,
                cruiseCommand,
                alignment * goalSlowdown
            );
            state = "following";
        }

        PublishDrive(linearCommand, steeringCommand);
    }

    void AdvanceReachedWaypoints(IReadOnlyList<Vector3> route, Vector3 position)
    {
        while (waypointIndex < route.Count)
        {
            Vector3 waypoint = route[waypointIndex];
            waypoint.y = 0f;
            float tolerance =
                waypointIndex == route.Count - 1 ? finalGoalTolerance : waypointTolerance;

            if (Vector3.Distance(position, waypoint) > tolerance)
            {
                break;
            }

            waypointIndex++;
        }
    }

    int SelectLookAheadWaypoint(IReadOnlyList<Vector3> route, Vector3 position)
    {
        int selected = waypointIndex;
        while (selected + 1 < route.Count)
        {
            Vector3 next = route[selected + 1];
            next.y = 0f;
            if (Vector3.Distance(position, next) > lookAheadDistance)
            {
                break;
            }

            selected++;
        }

        return selected;
    }

    bool ObstacleDetected()
    {
        if (!stopForObstacleSensors || sensors == null)
        {
            return false;
        }

        if (sensors.ultrasonicDist <= ultrasonicStop01)
        {
            return true;
        }

        return stopForFrontIr && (sensors.leftIR == 1 || sensors.rightIR == 1);
    }

    bool FreshGripperIrDetected()
    {
        if (sensors == null)
        {
            return false;
        }

        bool streamReady = !sensors.useRealSensors || sensors.RealSensorsFresh;
        return streamReady && sensors.gripperIR == 1;
    }

    Vector3 ResolveDriveForward()
    {
        float movementOffset = tracks != null ? tracks.movementYawOffset : 0f;
        return Vector3.ProjectOnPlane(
            Quaternion.Euler(0f, movementOffset, 0f) * transform.forward,
            Vector3.up
        ).normalized;
    }

    void PublishDrive(float linear, float steering)
    {
        if (tracks != null)
        {
            tracks.SetCommand(linear, steering);
        }

        if (rosBridge != null)
        {
            rosBridge.PublishCommand(linear, steering);
        }
    }

    void StopDrive()
    {
        linearCommand = 0f;
        steeringCommand = 0f;

        if (tracks != null)
        {
            tracks.SetCommand(0f, 0f);
        }

        if (rosBridge != null)
        {
            rosBridge.PublishStop();
        }
    }

    void CompleteRoute()
    {
        routeActive = false;
        state = "goal reached";
        StopDrive();

        bool handedOff = handoffToRobotBrainAtGoal && robotBrain != null;
        if (handedOff)
        {
            robotBrain.isMovementEnabled = true;
            robotBrain.enabled = true;
            state = "goal reached; YOLO handoff";
            ownsControl = false;
        }
        else
        {
            RestoreManualControl();
        }

        Debug.Log(
            $"[RoboMarvelPathFollower] Route {routeSequence} complete. {state}",
            this
        );
    }

    void RestoreManualControl()
    {
        if (!ownsControl)
        {
            return;
        }

        if (rosBridge != null)
        {
            rosBridge.manualTeleopEnabled = manualTeleopWasEnabled;
        }

        if (robotBrain != null &&
            disableRobotBrainWhileFollowing)
        {
            robotBrain.isMovementEnabled = brainMovementWasEnabled;
            robotBrain.enabled = brainWasEnabled;
        }

        ownsControl = false;
    }
}
