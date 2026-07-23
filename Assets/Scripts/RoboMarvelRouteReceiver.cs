using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable]
public sealed class RoboMarvelRoutePoint
{
    public float x;
    public float y;
    public float yaw;
}

[Serializable]
public sealed class RoboMarvelRoutePacket
{
    public string type;
    public int seq;
    public double stamp;
    public string frame_id;
    public RoboMarvelRoutePoint[] poses;
}

[DisallowMultipleComponent]
public sealed class RoboMarvelRouteReceiver : MonoBehaviour
{
    [Header("UDP input")]
    [Range(1024, 65535)] public int udpPort = 5006;
    public bool listenOnStart = true;
    [Range(1, 64)] public int maxQueuedPackets = 4;

    [Header("Route alignment")]
    [Tooltip("Usually the root transform of Xiao-r GFS-X_1.")]
    public Transform routeOrigin;
    public TrackController tracks;
    [Tooltip("Rotate the received path so its first segment matches the GFS-X drive direction.")]
    public bool alignFirstSegmentToRobotForward = true;
    [Tooltip("Use if the shared map has the opposite lateral direction.")]
    public bool invertLateralAxis;
    public float additionalYawOffsetDegrees;
    [Min(0.001f)] public float coordinateScale = 1f;
    [Min(0f)] public float minimumPointSpacing = 0.10f;

    [Header("Diagnostics (read only)")]
    [SerializeField] bool listenerRunning;
    [SerializeField] string listenerStatus = "Stopped";
    [SerializeField] string lastSender = "-";
    [SerializeField] string routeFrame = "-";
    [SerializeField] int lastSequence = -1;
    [SerializeField] int packetsReceived;
    [SerializeField] int routePointCount;
    [SerializeField] float lastPacketAge = -1f;
    [SerializeField] string lastError = "";

    readonly ConcurrentQueue<string> packetQueue = new ConcurrentQueue<string>();
    readonly List<Vector3> worldWaypoints = new List<Vector3>();
    UdpClient udpClient;
    Thread listenerThread;
    volatile bool stopRequested;
    float lastPacketTime = -1f;

    public event Action RouteUpdated;
    public IReadOnlyList<Vector3> WorldWaypoints => worldWaypoints;
    public bool HasRoute => worldWaypoints.Count >= 2;
    public int LastSequence => lastSequence;
    public bool ListenerRunning => listenerRunning;
    public float LastPacketAge => lastPacketAge;

    void Awake()
    {
        if (routeOrigin == null)
        {
            routeOrigin = transform;
        }

        if (tracks == null)
        {
            tracks = GetComponent<TrackController>();
        }
    }

    void Start()
    {
        if (listenOnStart)
        {
            StartListening();
        }
    }

    void Update()
    {
        if (lastPacketTime >= 0f)
        {
            lastPacketAge = Time.unscaledTime - lastPacketTime;
        }

        string newestPacket = null;
        while (packetQueue.TryDequeue(out string packet))
        {
            newestPacket = packet;
        }

        if (!string.IsNullOrEmpty(newestPacket))
        {
            ApplyPacket(newestPacket);
        }
    }

    void OnDestroy()
    {
        StopListening();
    }

    public void StartListening()
    {
        if (listenerThread != null && listenerThread.IsAlive)
        {
            return;
        }

        try
        {
            stopRequested = false;
            udpClient = new UdpClient(udpPort);
            udpClient.Client.ReceiveTimeout = 500;
            listenerThread = new Thread(ListenLoop)
            {
                IsBackground = true,
                Name = "RoboMarvel route UDP listener"
            };
            listenerThread.Start();
            listenerRunning = true;
            listenerStatus = $"Listening on UDP {udpPort}";
            lastError = "";
        }
        catch (Exception exception)
        {
            listenerRunning = false;
            listenerStatus = "Failed";
            lastError = exception.Message;
            Debug.LogError($"[RoboMarvelRouteReceiver] {exception.Message}", this);
        }
    }

    public void StopListening()
    {
        stopRequested = true;

        try
        {
            udpClient?.Close();
        }
        catch
        {
            // Socket may already be closed during application shutdown.
        }

        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join(1000);
        }

        listenerThread = null;
        udpClient = null;
        listenerRunning = false;
        listenerStatus = "Stopped";
    }

    void ListenLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        while (!stopRequested)
        {
            try
            {
                byte[] bytes = udpClient.Receive(ref remoteEndPoint);
                string packet = Encoding.UTF8.GetString(bytes);

                while (packetQueue.Count >= maxQueuedPackets &&
                       packetQueue.TryDequeue(out _))
                {
                }

                packetQueue.Enqueue(packet);
                lastSender = remoteEndPoint.Address.ToString();
                packetsReceived++;
            }
            catch (SocketException exception)
            {
                if (exception.SocketErrorCode != SocketError.TimedOut &&
                    exception.SocketErrorCode != SocketError.Interrupted &&
                    !stopRequested)
                {
                    lastError = exception.Message;
                }
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception exception)
            {
                if (!stopRequested)
                {
                    lastError = exception.Message;
                }
            }
        }
    }

    void ApplyPacket(string json)
    {
        RoboMarvelRoutePacket packet;
        try
        {
            packet = JsonUtility.FromJson<RoboMarvelRoutePacket>(json);
        }
        catch (Exception exception)
        {
            lastError = $"JSON: {exception.Message}";
            return;
        }

        if (packet == null ||
            packet.type != "nav_path" ||
            packet.poses == null ||
            packet.poses.Length < 2)
        {
            lastError = "Packet is not a nav_path with at least two poses.";
            return;
        }

        bool isNewRoute = packet.seq != lastSequence;
        if (isNewRoute)
        {
            BuildWorldRoute(packet);
        }

        routeFrame = string.IsNullOrEmpty(packet.frame_id) ? "-" : packet.frame_id;
        lastSequence = packet.seq;
        lastPacketTime = Time.unscaledTime;
        lastPacketAge = 0f;
        lastError = "";
        if (isNewRoute)
        {
            RouteUpdated?.Invoke();
        }
    }

    void BuildWorldRoute(RoboMarvelRoutePacket packet)
    {
        worldWaypoints.Clear();

        RoboMarvelRoutePoint first = packet.poses[0];
        List<Vector3> localPoints = new List<Vector3>(packet.poses.Length);

        foreach (RoboMarvelRoutePoint pose in packet.poses)
        {
            float rosForward = (pose.x - first.x) * coordinateScale;
            float rosLeft = (pose.y - first.y) * coordinateScale;
            float unityRight = invertLateralAxis ? rosLeft : -rosLeft;
            localPoints.Add(new Vector3(unityRight, 0f, rosForward));
        }

        Quaternion alignment = Quaternion.Euler(0f, additionalYawOffsetDegrees, 0f);
        if (alignFirstSegmentToRobotForward)
        {
            Vector3 firstDirection = FindFirstDirection(localPoints);
            Vector3 robotForward = ResolveRobotForward();
            if (firstDirection.sqrMagnitude > 0.0001f &&
                robotForward.sqrMagnitude > 0.0001f)
            {
                alignment =
                    Quaternion.FromToRotation(firstDirection, robotForward) *
                    Quaternion.Euler(0f, additionalYawOffsetDegrees, 0f);
            }
        }
        else if (routeOrigin != null)
        {
            alignment =
                Quaternion.Euler(0f, routeOrigin.eulerAngles.y + additionalYawOffsetDegrees, 0f);
        }

        Vector3 origin = routeOrigin != null ? routeOrigin.position : transform.position;
        float minimumSpacingSquared = minimumPointSpacing * minimumPointSpacing;

        foreach (Vector3 point in localPoints)
        {
            Vector3 worldPoint = origin + alignment * point;
            if (worldWaypoints.Count == 0 ||
                (worldPoint - worldWaypoints[worldWaypoints.Count - 1]).sqrMagnitude >=
                minimumSpacingSquared)
            {
                worldWaypoints.Add(worldPoint);
            }
        }

        Vector3 finalPoint = origin + alignment * localPoints[localPoints.Count - 1];
        if (worldWaypoints.Count == 0 ||
            (finalPoint - worldWaypoints[worldWaypoints.Count - 1]).sqrMagnitude > 0.0001f)
        {
            worldWaypoints.Add(finalPoint);
        }

        routePointCount = worldWaypoints.Count;
    }

    Vector3 ResolveRobotForward()
    {
        Transform origin = routeOrigin != null ? routeOrigin : transform;
        float movementOffset = tracks != null ? tracks.movementYawOffset : 0f;
        return Vector3.ProjectOnPlane(
            Quaternion.Euler(0f, movementOffset, 0f) * origin.forward,
            Vector3.up
        ).normalized;
    }

    static Vector3 FindFirstDirection(List<Vector3> points)
    {
        if (points.Count < 2)
        {
            return Vector3.zero;
        }

        Vector3 start = points[0];
        for (int index = 1; index < points.Count; index++)
        {
            Vector3 direction = Vector3.ProjectOnPlane(points[index] - start, Vector3.up);
            if (direction.sqrMagnitude > 0.0025f)
            {
                return direction.normalized;
            }
        }

        return Vector3.zero;
    }

    void OnDrawGizmosSelected()
    {
        if (worldWaypoints.Count < 2)
        {
            return;
        }

        Gizmos.color = Color.cyan;
        for (int index = 0; index < worldWaypoints.Count - 1; index++)
        {
            Gizmos.DrawLine(worldWaypoints[index], worldWaypoints[index + 1]);
            Gizmos.DrawSphere(worldWaypoints[index], 0.04f);
        }

        Gizmos.color = Color.green;
        Gizmos.DrawSphere(worldWaypoints[worldWaypoints.Count - 1], 0.08f);
    }
}
