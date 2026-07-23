using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Threading;
using UnityEngine;

/// <summary>
/// Minimal MJPEG viewer for the camera_stream_team1 server.
/// It displays the live feed in Unity without adding buttons or a Canvas.
/// The rover's camera_stream_team1.py serves the MJPEG stream at the root path.
/// </summary>
public class MjpegStreamViewer : MonoBehaviour
{
    [Header("Robot camera")]
    public string streamUrl = "http://192.168.2.155:8080/";
    public bool connectOnStart = true;
    [Min(1)] public int requestTimeoutSeconds = 5;
    [Min(0.5f)] public float reconnectDelaySeconds = 2f;

    [Header("Unity display")]
    public bool showOnGUI = true;
    public Rect displayRect = new Rect(20f, 20f, 480f, 270f);
    public bool keepAspect = true;

    [Header("YOLO overlay")]
    public RealVision realVision;
    public bool showDetections = true;
    [Range(0f, 1f)] public float minimumConfidence = 0.2f;
    [Range(0f, 1f)] public float ballMinimumConfidence = 0.12f;
    [Range(0f, 1f)] public float cubeMinimumConfidence = 0.05f;
    [Min(1f)] public float boxThickness = 3f;
    public bool showConfidence = true;
    public Color ballBoxColor = new Color(1f, 0.5f, 0f, 1f);
    public Color cubeBoxColor = new Color(0.95f, 0.1f, 0.1f, 1f);
    public Color otherBoxColor = Color.cyan;

    readonly object frameLock = new object();
    readonly object requestLock = new object();
    Thread readerThread;
    HttpWebRequest activeRequest;
    byte[] pendingJpeg;
    Texture2D texture;
    string status = "Not connected";
    string lastReportedStatus;
    bool stopRequested;
    GUIStyle detectionLabelStyle;

    void Start()
    {
        if (realVision == null)
        {
            realVision = GetComponent<RealVision>();
        }

        if (realVision == null)
        {
            realVision = GetComponentInParent<RealVision>();
        }

        if (connectOnStart)
        {
            Connect();
        }
    }

    void Update()
    {
        if (status != lastReportedStatus)
        {
            lastReportedStatus = status;
            Debug.Log($"MjpegStreamViewer: {status} ({streamUrl})", this);
        }

        byte[] nextFrame = null;
        lock (frameLock)
        {
            if (pendingJpeg != null)
            {
                nextFrame = pendingJpeg;
                pendingJpeg = null;
            }
        }

        if (nextFrame == null)
        {
            return;
        }

        if (texture == null)
        {
            texture = new Texture2D(2, 2, TextureFormat.RGB24, false);
        }

        texture.LoadImage(nextFrame, false);
    }

    public void Connect()
    {
        StopReader();

        if (string.IsNullOrWhiteSpace(streamUrl))
        {
            status = "Stream URL is empty";
            return;
        }

        // camera_stream_team1.py serves MJPEG at the root path. Older scene
        // settings used the mjpg-streamer query and receive HTTP 404.
        streamUrl = NormalizeTeam1StreamUrl(streamUrl);

        stopRequested = false;
        status = "Connecting...";
        readerThread = new Thread(ReadStream)
        {
            IsBackground = true,
            Name = "GFS-X MJPEG reader"
        };
        readerThread.Start();
    }

    static string NormalizeTeam1StreamUrl(string url)
    {
        if (!Uri.TryCreate(url, UriKind.Absolute, out Uri parsed))
        {
            return url;
        }

        if (parsed.Query.IndexOf("action=stream", StringComparison.OrdinalIgnoreCase) < 0)
        {
            return url;
        }

        UriBuilder builder = new UriBuilder(parsed)
        {
            Query = string.Empty
        };
        return builder.Uri.AbsoluteUri;
    }

    void ReadStream()
    {
        while (!stopRequested)
        {
            try
            {
                HttpWebRequest request = WebRequest.CreateHttp(streamUrl);
                request.Timeout = requestTimeoutSeconds * 1000;
                request.ReadWriteTimeout = requestTimeoutSeconds * 1000;
                request.KeepAlive = true;

                lock (requestLock)
                {
                    activeRequest = request;
                }

                using (WebResponse response = request.GetResponse())
                using (Stream stream = response.GetResponseStream())
                {
                    status = "Connected";

                    if (stream == null)
                    {
                        status = "No response stream";
                    }
                    else
                    {
                        ReadJpegFrames(stream);
                    }
                }
            }
            catch (WebException exception)
            {
                if (!stopRequested)
                {
                    status = "Connection failed: " + exception.Message;
                }
            }
            catch (Exception exception)
            {
                if (!stopRequested)
                {
                    status = "Stream error: " + exception.Message;
                }
            }
            finally
            {
                lock (requestLock)
                {
                    activeRequest = null;
                }
            }

            if (!stopRequested)
            {
                Thread.Sleep((int)(reconnectDelaySeconds * 1000f));
            }
        }
    }

    void ReadJpegFrames(Stream stream)
    {
        List<byte> buffer = new List<byte>(256 * 1024);
        byte[] chunk = new byte[16 * 1024];

        while (!stopRequested)
        {
            int count = stream.Read(chunk, 0, chunk.Length);
            if (count <= 0)
            {
                status = "Stream closed";
                return;
            }

            for (int i = 0; i < count; i++)
            {
                buffer.Add(chunk[i]);
            }

            while (TryExtractJpeg(buffer, out byte[] jpeg))
            {
                lock (frameLock)
                {
                    pendingJpeg = jpeg;
                }
            }

            if (buffer.Count > 4 * 1024 * 1024)
            {
                buffer.RemoveRange(0, buffer.Count - 1024 * 1024);
            }
        }
    }

    static bool TryExtractJpeg(List<byte> buffer, out byte[] jpeg)
    {
        jpeg = null;
        int start = FindMarker(buffer, 0, 0xff, 0xd8);
        if (start < 0)
        {
            if (buffer.Count > 1)
            {
                buffer.RemoveRange(0, buffer.Count - 1);
            }

            return false;
        }

        int end = FindMarker(buffer, start + 2, 0xff, 0xd9);
        if (end < 0)
        {
            if (start > 0)
            {
                buffer.RemoveRange(0, start);
            }

            return false;
        }

        int length = end + 2 - start;
        jpeg = buffer.GetRange(start, length).ToArray();
        buffer.RemoveRange(0, end + 2);
        return true;
    }

    static int FindMarker(List<byte> buffer, int from, byte first, byte second)
    {
        for (int i = from; i + 1 < buffer.Count; i++)
        {
            if (buffer[i] == first && buffer[i + 1] == second)
            {
                return i;
            }
        }

        return -1;
    }

    void OnGUI()
    {
        if (!showOnGUI || texture == null)
        {
            return;
        }

        ScaleMode scaleMode = keepAspect ? ScaleMode.ScaleToFit : ScaleMode.StretchToFill;
        GUI.DrawTexture(displayRect, texture, scaleMode, false);

        if (showDetections && realVision != null)
        {
            DrawDetections(GetDisplayedTextureRect());
        }
    }

    Rect GetDisplayedTextureRect()
    {
        if (!keepAspect || texture == null || texture.width <= 0 || texture.height <= 0)
        {
            return displayRect;
        }

        float textureAspect = (float)texture.width / texture.height;
        float displayAspect = displayRect.width / Mathf.Max(1f, displayRect.height);

        if (textureAspect > displayAspect)
        {
            float height = displayRect.width / textureAspect;
            return new Rect(
                displayRect.x,
                displayRect.y + (displayRect.height - height) * 0.5f,
                displayRect.width,
                height
            );
        }

        float width = displayRect.height * textureAspect;
        return new Rect(
            displayRect.x + (displayRect.width - width) * 0.5f,
            displayRect.y,
            width,
            displayRect.height
        );
    }

    void DrawDetections(Rect imageRect)
    {
        YoloDetectionPacket[] detections = realVision.Detections;
        if (detections == null || detections.Length == 0)
        {
            return;
        }

        EnsureDetectionLabelStyle();

        foreach (YoloDetectionPacket detection in detections)
        {
            if (detection == null || detection.conf < DetectionThreshold(detection.class_id))
            {
                continue;
            }

            Rect boxRect = new Rect(
                imageRect.x + detection.x * imageRect.width,
                imageRect.y + detection.y * imageRect.height,
                detection.width * imageRect.width,
                detection.height * imageRect.height
            );

            if (boxRect.width < 1f || boxRect.height < 1f)
            {
                continue;
            }

            Color color = DetectionColor(detection.class_id);
            DrawBoxOutline(boxRect, color, boxThickness);

            string label = string.IsNullOrWhiteSpace(detection.label)
                ? $"class {detection.class_id}"
                : detection.label;
            if (showConfidence)
            {
                label += $" {detection.conf:0.00}";
            }

            Vector2 labelSize = detectionLabelStyle.CalcSize(new GUIContent(label));
            Rect labelRect = new Rect(
                boxRect.x,
                Mathf.Max(imageRect.y, boxRect.y - labelSize.y - 4f),
                labelSize.x + 10f,
                labelSize.y + 4f
            );

            Color previousColor = GUI.color;
            GUI.color = new Color(color.r, color.g, color.b, 0.9f);
            GUI.DrawTexture(labelRect, Texture2D.whiteTexture);
            GUI.color = Color.white;
            GUI.Label(
                new Rect(labelRect.x + 5f, labelRect.y + 2f, labelSize.x, labelSize.y),
                label,
                detectionLabelStyle
            );
            GUI.color = previousColor;
        }
    }

    float DetectionThreshold(int classId)
    {
        if (classId == 0)
        {
            return ballMinimumConfidence;
        }

        if (classId == 1)
        {
            return cubeMinimumConfidence;
        }

        return minimumConfidence;
    }

    void EnsureDetectionLabelStyle()
    {
        if (detectionLabelStyle != null)
        {
            return;
        }

        detectionLabelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 14,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
            clipping = TextClipping.Clip
        };
        detectionLabelStyle.normal.textColor = Color.white;
    }

    static void DrawBoxOutline(Rect rect, Color color, float thickness)
    {
        Color previousColor = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, thickness), Texture2D.whiteTexture);
        GUI.DrawTexture(
            new Rect(rect.x, rect.yMax - thickness, rect.width, thickness),
            Texture2D.whiteTexture
        );
        GUI.DrawTexture(new Rect(rect.x, rect.y, thickness, rect.height), Texture2D.whiteTexture);
        GUI.DrawTexture(
            new Rect(rect.xMax - thickness, rect.y, thickness, rect.height),
            Texture2D.whiteTexture
        );
        GUI.color = previousColor;
    }

    Color DetectionColor(int classId)
    {
        if (classId == 0)
        {
            return ballBoxColor;
        }

        if (classId == 1)
        {
            return cubeBoxColor;
        }

        return otherBoxColor;
    }

    void OnDestroy()
    {
        StopReader();

        if (texture != null)
        {
            Destroy(texture);
        }
    }

    void StopReader()
    {
        stopRequested = true;

        lock (requestLock)
        {
            if (activeRequest != null)
            {
                activeRequest.Abort();
                activeRequest = null;
            }
        }

        if (readerThread != null && readerThread.IsAlive && readerThread != Thread.CurrentThread)
        {
            readerThread.Join(250);
        }

        readerThread = null;
    }
}
