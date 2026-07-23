using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable]
public class YoloDataPacket
{
    public float angle;
    public float distance;
    public float sees;
    public float conf;
    public float w;
    public float h;
    public float frame_w;
    public float frame_h;
    public int seq;
    public double timestamp;
    public int target_class_id = -1;
    public string target_label;
    public YoloDetectionPacket[] detections;
}

[Serializable]
public class YoloDetectionPacket
{
    public int class_id;
    public string label;
    public float conf;
    public float x;
    public float y;
    public float width;
    public float height;
}

[DisallowMultipleComponent]
public class RealVision : MonoBehaviour
{
    [Header("UDP input")]
    [Range(1, 65535)] public int udpPort = 5005;
    public bool listenOnStart = true;
    [Min(0.05f)] public float staleAfterSeconds = 1.0f;
    [Range(1, 64)] public int maxQueuedPackets = 8;
    [Min(0.1f)] public float maxViewDistance = 6f;

    [Header("Live detection (read only)")]
    [SerializeField] bool isTargetVisible;
    [SerializeField, Range(-1f, 1f)] float targetX;
    [SerializeField, Range(0f, 1f)] float distance01 = 1f;
    [SerializeField, Range(0f, 1f)] float boxHeight01;
    [SerializeField, Range(0f, 1f)] float confidence;
    [SerializeField] float boxWidthPixels;
    [SerializeField] float boxHeightPixels;
    [SerializeField] int detectionCount;
    [SerializeField] int targetClassId = -1;
    [SerializeField] string targetLabel = "";

    [Header("Diagnostics (read only)")]
    [SerializeField] bool listenerRunning;
    [SerializeField] string listenerStatus = "Stopped";
    [SerializeField] string lastSender = "-";
    [SerializeField] int packetsReceived;
    [SerializeField] int lastSequence = -1;
    [SerializeField] int outOfOrderPackets;
    [SerializeField] float packetRateHz;
    [SerializeField] float lastPacketAge = -1f;
    [SerializeField] string lastError = "";

    readonly ConcurrentQueue<string> packetQueue = new ConcurrentQueue<string>();
    readonly ConcurrentQueue<string> statusQueue = new ConcurrentQueue<string>();
    readonly object socketLock = new object();

    Thread listenerThread;
    UdpClient udpClient;
    volatile bool stopRequested;
    float lastPacketTime = -1f;
    float lastKnownTargetX;
    float rateWindowStarted;
    int packetsInRateWindow;
    YoloDetectionPacket[] detections = Array.Empty<YoloDetectionPacket>();
    double lastSourceTimestamp = -1.0;

    public bool IsTargetVisible => isTargetVisible;
    public float TargetX => targetX;

    // Matches SimulatedYoloCamera: 0 means close and 1 means far/not visible.
    public float Distance01 => distance01;
    public int TargetClassId => targetClassId;
    public string TargetLabel => targetLabel;
    public YoloDetectionPacket[] Detections => detections;

    public bool TryGetTarget(
        int requestedClassId,
        out float normalizedX,
        out float normalizedDistance,
        out float targetConfidence,
        out string label)
    {
        normalizedX = 0f;
        normalizedDistance = 1f;
        targetConfidence = 0f;
        label = "";

        if (requestedClassId < 0)
        {
            if (!isTargetVisible)
            {
                return false;
            }

            normalizedX = targetX;
            normalizedDistance = distance01;
            targetConfidence = confidence;
            label = targetLabel;
            return true;
        }

        // Some YOLO senders provide only the selected top-level target and no
        // detections array. Accept that packet when it is the requested class.
        if (isTargetVisible && targetClassId == requestedClassId)
        {
            normalizedX = targetX;
            normalizedDistance = distance01;
            targetConfidence = confidence;
            label = string.IsNullOrWhiteSpace(targetLabel)
                ? TargetLabelForClass(requestedClassId)
                : targetLabel;
            return true;
        }

        YoloDetectionPacket best = null;
        float bestScore = -1f;
        foreach (YoloDetectionPacket detection in detections)
        {
            if (detection == null || detection.class_id != requestedClassId)
            {
                continue;
            }

            float area = Mathf.Clamp01(detection.width) * Mathf.Clamp01(detection.height);
            float score = area * (0.5f + 0.5f * Mathf.Clamp01(detection.conf));
            if (score <= bestScore)
            {
                continue;
            }

            best = detection;
            bestScore = score;
        }

        if (best == null)
        {
            return false;
        }

        float centerX = Mathf.Clamp01(best.x + best.width * 0.5f);
        normalizedX = Mathf.Clamp(centerX * 2f - 1f, -1f, 1f);
        normalizedDistance = 1f - Mathf.Clamp01(best.height);
        targetConfidence = Mathf.Clamp01(best.conf);
        label = string.IsNullOrWhiteSpace(best.label)
            ? TargetLabelForClass(requestedClassId)
            : best.label;
        return true;
    }

    // Compatibility names from the Practice 7 guide.
    public bool seesBall => isTargetVisible;
    public float normalizedAngle => targetX;
    public float normalizedDistance => distance01;
    public float lastKnownBallDirection => lastKnownTargetX;
    public float yoloConfidence => confidence;
    public float bboxWidth => boxWidthPixels;
    public float bboxHeight => boxHeightPixels;

    void OnEnable()
    {
        if (Application.isPlaying && listenOnStart)
        {
            StartListener();
        }
    }

    void Update()
    {
        while (statusQueue.TryDequeue(out string statusMessage))
        {
            if (statusMessage.StartsWith("ERROR:", StringComparison.Ordinal))
            {
                lastError = statusMessage.Substring(6).Trim();
                listenerStatus = "Error";
            }
            else if (statusMessage.StartsWith("SENDER:", StringComparison.Ordinal))
            {
                lastSender = statusMessage.Substring(7).Trim();
            }
            else
            {
                listenerStatus = statusMessage;
            }
        }

        string newestJson = null;
        int drainedPackets = 0;
        while (packetQueue.TryDequeue(out string json))
        {
            newestJson = json;
            drainedPackets++;
        }

        if (newestJson != null)
        {
            ApplyPacket(newestJson);
            packetsReceived += drainedPackets;
            packetsInRateWindow += drainedPackets;
        }

        float now = Time.realtimeSinceStartup;
        if (rateWindowStarted <= 0f)
        {
            rateWindowStarted = now;
        }

        float rateWindowDuration = now - rateWindowStarted;
        if (rateWindowDuration >= 1f)
        {
            packetRateHz = packetsInRateWindow / rateWindowDuration;
            packetsInRateWindow = 0;
            rateWindowStarted = now;
        }

        if (lastPacketTime < 0f)
        {
            lastPacketAge = -1f;
            ClearVision();
        }
        else
        {
            lastPacketAge = now - lastPacketTime;
            if (lastPacketAge > staleAfterSeconds)
            {
                ClearVision();
            }
        }
    }

    public void StartListener()
    {
        if (listenerThread != null && listenerThread.IsAlive)
        {
            return;
        }

        StopListener();
        stopRequested = false;
        lastError = "";
        lastSequence = -1;
        lastSourceTimestamp = -1.0;
        lastKnownTargetX = 0f;
        listenerStatus = $"Starting UDP {udpPort}";
        listenerThread = new Thread(ListenLoop)
        {
            IsBackground = true,
            Name = "GFS-X YOLO UDP listener"
        };
        listenerThread.Start();
    }

    public void StopListener()
    {
        stopRequested = true;

        lock (socketLock)
        {
            if (udpClient != null)
            {
                udpClient.Close();
                udpClient = null;
            }
        }

        if (listenerThread != null && listenerThread.IsAlive && listenerThread != Thread.CurrentThread)
        {
            listenerThread.Join(500);
        }

        listenerThread = null;
        listenerRunning = false;
        listenerStatus = "Stopped";
        ClearVision();
    }

    void ListenLoop()
    {
        UdpClient localClient = null;

        try
        {
            localClient = new UdpClient();
            localClient.ExclusiveAddressUse = false;
            localClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            localClient.Client.ReceiveTimeout = 250;
            localClient.Client.Bind(new IPEndPoint(IPAddress.Any, udpPort));

            lock (socketLock)
            {
                udpClient = localClient;
            }

            listenerRunning = true;
            statusQueue.Enqueue($"Listening on UDP {udpPort}");

            IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
            string previousSender = null;

            while (!stopRequested)
            {
                try
                {
                    byte[] bytes = localClient.Receive(ref sender);
                    string json = Encoding.UTF8.GetString(bytes);

                    while (packetQueue.Count >= maxQueuedPackets && packetQueue.TryDequeue(out _))
                    {
                    }
                    packetQueue.Enqueue(json);

                    string senderText = sender.ToString();
                    if (senderText != previousSender)
                    {
                        previousSender = senderText;
                        statusQueue.Enqueue("SENDER: " + senderText);
                    }
                }
                catch (SocketException exception)
                    when (exception.SocketErrorCode == SocketError.TimedOut ||
                          exception.SocketErrorCode == SocketError.WouldBlock)
                {
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
            }
        }
        catch (Exception exception)
        {
            if (!stopRequested)
            {
                statusQueue.Enqueue("ERROR: " + exception.Message);
            }
        }
        finally
        {
            listenerRunning = false;
            localClient?.Close();
            lock (socketLock)
            {
                if (udpClient == localClient)
                {
                    udpClient = null;
                }
            }
        }
    }

    void ApplyPacket(string json)
    {
        try
        {
            YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);
            if (packet == null)
            {
                lastError = "Received an empty JSON packet";
                return;
            }

            if (packet.timestamp > 0.0 && packet.timestamp <= lastSourceTimestamp)
            {
                outOfOrderPackets++;
                return;
            }

            if (packet.timestamp > 0.0)
            {
                lastSourceTimestamp = packet.timestamp;
            }
            lastSequence = packet.seq;
            lastPacketTime = Time.realtimeSinceStartup;
            lastError = "";
            confidence = Mathf.Clamp01(packet.conf);
            boxWidthPixels = Mathf.Max(0f, packet.w);
            boxHeightPixels = Mathf.Max(0f, packet.h);
            boxHeight01 = Mathf.Clamp01(packet.distance);
            ApplyDetections(packet.detections);

            if (packet.sees > 0.5f)
            {
                isTargetVisible = true;
                targetX = Mathf.Clamp(packet.angle, -1f, 1f);
                lastKnownTargetX = targetX;
                targetClassId = packet.target_class_id;
                targetLabel = string.IsNullOrWhiteSpace(packet.target_label)
                    ? TargetLabelForClass(targetClassId)
                    : packet.target_label;

                // Python sends box height / frame height, which grows as the ball
                // approaches. RobotBrain was trained with the opposite convention.
                distance01 = 1f - boxHeight01;
            }
            else
            {
                SetTargetNotVisible();
            }
        }
        catch (Exception exception)
        {
            lastError = "Invalid YOLO JSON: " + exception.Message;
        }
    }

    void ApplyDetections(YoloDetectionPacket[] receivedDetections)
    {
        if (receivedDetections == null || receivedDetections.Length == 0)
        {
            detections = Array.Empty<YoloDetectionPacket>();
            detectionCount = 0;
            return;
        }

        foreach (YoloDetectionPacket detection in receivedDetections)
        {
            if (detection == null)
            {
                continue;
            }

            detection.conf = Mathf.Clamp01(detection.conf);
            detection.x = Mathf.Clamp01(detection.x);
            detection.y = Mathf.Clamp01(detection.y);
            detection.width = Mathf.Clamp(detection.width, 0f, 1f - detection.x);
            detection.height = Mathf.Clamp(detection.height, 0f, 1f - detection.y);

            if (string.IsNullOrWhiteSpace(detection.label))
            {
                detection.label = detection.class_id == 0
                    ? "ball"
                    : detection.class_id == 1
                        ? "cube"
                        : $"class {detection.class_id}";
            }
        }

        detections = receivedDetections;
        detectionCount = detections.Length;
    }

    void SetTargetNotVisible()
    {
        isTargetVisible = false;
        targetX = 0f;
        distance01 = 1f;
        targetClassId = -1;
        targetLabel = "";
    }

    static string TargetLabelForClass(int classId)
    {
        if (classId == 0) return "ball";
        if (classId == 1) return "cube";
        return classId >= 0 ? $"class {classId}" : "";
    }

    void ClearVision()
    {
        SetTargetNotVisible();
        detections = Array.Empty<YoloDetectionPacket>();
        detectionCount = 0;
    }

    void OnDisable()
    {
        StopListener();
    }

    void OnApplicationQuit()
    {
        StopListener();
    }

    void OnValidate()
    {
        udpPort = Mathf.Clamp(udpPort, 1, 65535);
        staleAfterSeconds = Mathf.Max(0.05f, staleAfterSeconds);
        maxQueuedPackets = Mathf.Clamp(maxQueuedPackets, 1, 64);
        maxViewDistance = Mathf.Max(0.1f, maxViewDistance);
    }
}

// Optional compatibility hook for the supplied RobotBrain.
// Leave this component off the robot unless per-step diagnostics are needed.
public class DiagnosticLogger : MonoBehaviour
{
    public void LogStep(
        int step,
        bool ballSeen,
        float ballAngle,
        float ballDistance,
        float ultrasonic,
        int leftIr,
        int rightIr,
        int gripperIr,
        float cameraYaw,
        float gas,
        float steering,
        float cameraInput,
        bool hasBall,
        int holdTicks,
        bool retrying,
        bool closeToBall,
        int blindApproachTicks,
        bool ballRecentlySeen,
        int holdWithoutIr,
        float displacementX,
        float displacementZ,
        float heading,
        float speed,
        float worldX,
        float worldZ,
        float yoloConfidence,
        float bboxWidth,
        float bboxHeight,
        float pwmLeft,
        float pwmRight)
    {
    }
}
