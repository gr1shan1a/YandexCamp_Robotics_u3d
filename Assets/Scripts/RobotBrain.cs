using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

[RequireComponent(typeof(TrackController))]
[RequireComponent(typeof(VirtualSensors))]
public class RobotBrain : Agent
{
    private enum RewardComponent
    {
        Approach = 0,
        Alignment = 1,
        Grab = 2,
        TimePenalty = 3,
        BlindApproach = 4,
        BlindPenalty = 5,
        CollisionPenalty = 6,
        ReversePenalty = 7,
        ActionPenalty = 8,
        Other = 9,
        Count = 10
    }

    private enum EpisodeEndReason
    {
        Success,
        Timeout,
        Boundary,
        Collision,
        Stuck,
        Other
    }

    private const int REWARD_COMPONENT_COUNT = (int)RewardComponent.Count;
    private static readonly string[] RewardComponentNames =
    {
        "Approach",
        "Alignment",
        "Grab",
        "TimePenalty",
        "BlindApproach",
        "BlindPenalty",
        "CollisionPenalty",
        "ReversePenalty",
        "ActionPenalty",
        "Other"
    };

    private TrackController track;
    private VirtualSensors sensors;
    private Rigidbody rb;

    [Header("General Settings")]
    public bool isTraining = false;
    public bool isMovementEnabled = false; // "Ручник" для калибровки
    public bool enableVerboseLogging = true;
    [Tooltip("Пишет компактный CSV: одна строка на завершенный training-эпизод.")]
    public bool enableTrainingCsvLog = true;
    [SerializeField] private bool debugRewardTracking = false;

    [Header("Ball Spawn")]
    public float ballSpawnMinRadius = 0.35f;
    public float ballSpawnMaxRadius = 0.8f;
    private const float BALL_SPAWN_FLOOR_RAY_HEIGHT = 2.0f;
    private const float BALL_SPAWN_FLOOR_RAY_DISTANCE = 5.0f;

    [Header("Inference Test Spawn")]
    [Tooltip("При inference-сбросе ставить мяч рядом с роботом для быстрого теста модели.")]
    public bool resetBallNearRobotForInference = true;
    [Tooltip("Локальные координаты мяча относительно робота: X=вправо, Y=вверх, Z=вперед.")]
    public Vector3 inferenceBallLocalOffset = new Vector3(0f, 0.2f, 0.8f);
    public bool logInferenceActions = false;

    [Header("Inference Stats")]
    public int successfulPickups = 0;
    private bool wasHoldingBallLastStep = false;
    private int inferenceActionLogCounter = 0;

    [Header("Vision Approach Override (Inference)")]
    [Tooltip("When YOLO sees a ball or cube, use deterministic visual servoing instead of waiting for a useful policy action.")]
    public bool enableVisionApproachOverride = true;
    [Tooltip("Automatically release the movement lock when running inference.")]
    public bool autoEnableMovementForInference = true;
    [Tooltip("Physical autonomous mode: ignore cube detections and approach YOLO class 0 (ball) only.")]
    public bool ballOnlyAutonomousInference = true;
    [Tooltip("Automatically subscribe to /sensor/data in inference so the physical gripper IR cannot be disabled accidentally.")]
    public bool forceRealSensorsForInference = true;
    [Min(0.01f)] public float approachCruiseSpeedMps = 0.18f;
    [Min(0.01f)] public float approachCreepSpeedMps = 0.06f;
    [Min(0.05f)] public float approachSteeringGainRad = 1.4f;
    [Min(0.05f)] public float approachMaxAngularSpeedRad = 0.8f;
    [Range(0.01f, 0.5f)] public float visualCenterTolerance = 0.12f;
    [Range(0.05f, 1f)] public float rotateInPlaceHeadingError = 0.30f;
    [Range(0.01f, 1f)]
    [Tooltip("Image-edge error converted to camera-relative heading. 0.22 is about half of a 40 degree FOV.")]
    public float imageErrorToHeading = 0.22f;
    [Range(0f, 1f)]
    [Tooltip("Start slowing when Distance01 drops below this value. Distance01 is 0 near and 1 far.")]
    public float approachSlowDistance01 = 0.78f;
    [Range(0f, 1f)]
    [Tooltip("A centered target at or below this distance may be grabbed without waiting for gripper IR.")]
    public float visualGrabDistance01 = 0.60f;
    [Min(0f)] public float visualGrabConfirmSeconds = 0.25f;
    [Min(0f)] public float blindApproachSeconds = 1.1f;
    [Range(0f, 1f)]
    [Tooltip("The target must have been at least this close before a blind grab attempt.")]
    public float blindGrabStartDistance01 = 0.72f;
    [Min(0f)]
    [Tooltip("How long to creep after the close target disappears below the camera before closing the claw.")]
    public float blindGrabDelaySeconds = 0.55f;
    public bool allowVisionOnlyGrab = true;
    [Tooltip("Send command 1 once when a target is acquired so the physical arm is lowered and the claw is open.")]
    public bool preparePhysicalGripperOnTarget = true;
    [Tooltip("Enable if the real robot turns away from a target shown on the right side of the image.")]
    public bool invertVisionSteering = false;

    [Header("Vision Approach Diagnostics (read only)")]
    [SerializeField] private bool visionApproachActive;
    [SerializeField] private string visionApproachState = "idle";
    [SerializeField] private float visionHeadingError;
    [SerializeField] private float visionDriveCommand;
    [SerializeField] private float visionSteeringCommand;

    [Header("Inference Grab Safety")]
    [Tooltip("In inference, ensure a per-frame gripper-IR safety component is present. " +
             "This closes the physical claw even between ML-Agent decision steps.")]
    public bool ensureIndependentIrAutoClose = true;
    [Tooltip("On the real robot, accept gripper IR only after the target was seen close enough. " +
             "RealVision distance is 0 when close and 1 when far.")]
    public bool requireCloseVisualTargetForInferenceGrab = true;
    [Range(0f, 1f)]
    [Tooltip("Largest normalized vision distance at which a real grab may start. " +
             "0.65 means the target box is roughly 35% of the camera-frame height.")]
    public float maxInferenceGrabDistance01 = 0.65f;

    [Header("Automatic Physical Grab")]
    [Tooltip("After a close, centered visual approach, close the physical claw even if gripper IR has not fired.")]
    public bool forceGrabAfterVisualApproach = true;
    [Min(0f)]
    [Tooltip("Wait for command 1 to lower the arm and open the claw before driving toward the target.")]
    public float armPrepareWaitSeconds = 0.7f;
    [Min(0.05f)]
    [Tooltip("Repeat the final S4 close command at this interval so the physical command cannot be missed.")]
    public float physicalCloseRepeatInterval = 0.15f;
    [Min(0.1f)]
    [Tooltip("How long to keep repeating the final physical claw command.")]
    public float physicalCloseRepeatSeconds = 1.2f;

    [Header("Target Pickup")]
    [Tooltip("Automatically approach whichever supported YOLO target is selected: ball=0 or cube=1.")]
    public bool approachBallAndCube = false;
    [Range(-1, 1)]
    [Tooltip("Used only when Approach Ball And Cube is off: -1 any, 0 ball, 1 cube.")]
    public int inferenceTargetClassId = 0;
    [Tooltip("Keep creeping toward a cube until the physical gripper IR confirms it is between the jaws.")]
    public bool requireGripperIrForCube = true;
    [Tooltip("Keep creeping toward a ball until the physical gripper IR confirms it is between the jaws.")]
    public bool requireGripperIrForBall = true;
    [Min(0f)]
    [Tooltip("How long a close cube may remain below the camera while the robot creeps toward the gripper IR.")]
    public float cubeBlindApproachSeconds = 3f;
    [Min(0f)]
    [Tooltip("How long a close ball may remain below the camera while the robot creeps toward the gripper IR.")]
    public float ballBlindApproachSeconds = 3f;
    [Range(0f, 1f)]
    [Tooltip("Partial closure used by the Unity twin for a cube. Physical S4 uses ROSBridge Cube Grip Angle.")]
    public float unityCubeJawClosure = 0.5f;

    [Header("Latency Simulation (Sim-to-Real)")]
    [Tooltip("Минимальная задержка действий (в шагах FixedUpdate). 1 шаг ≈ 20мс при 50Hz.")]
    public int minActionLatency = 8;
    [Tooltip("Максимальная задержка действий (рандомизируется каждый эпизод).")]
    public int maxActionLatency = 13;
    [Tooltip("Задержка сенсоров (шаги). Имитирует запаздывание ROS-топиков.")]
    public int sensorLatency = 2;
    private int currentActionLatency = 3;

    // Буфер задержки действий (Circular Queue)
    private Queue<float[]> actionBuffer = new Queue<float[]>();
    private Queue<int[]> discreteActionBuffer = new Queue<int[]>();
    // Буфер задержки сенсоров
    private Queue<float[]> sensorBuffer = new Queue<float[]>();
    private float[] delayedSensors = new float[4]; // UZ, L_IR, R_IR, CLAW_IR

    [Header("Stage 4 Components")]
    public RealVision realVision;
    public VirtualCamera virtualCamera;
    public SimulatedYoloCamera simulatedYolo; // ЗАМЕНА virtualCamera для обучения (YOLO-идентичные наблюдения)
    public ROSBridge rosBridge;

    [Header("Stage 6 Components")]
    public Transform cameraPivot; // Пустышка, на которой висит камера для вращения
    private float currentCameraYaw = 0f;

    [Header("Stage 3 Components")]
    public GripperController gripper;
    private DiagnosticLogger diagLogger;
    private GripperIRAutoClose gripperIrAutoClose;

    [Header("Parallel Training Setup")]
    public GameObject ballPrefab; // Префаб мяча
    private GameObject spawnedBall; // Ссылка на мяч (созданный или найденный)
    [Tooltip("Если включено, эпизод не рандомизирует мяч, а возвращает его в позицию, заданную в сцене.")]
    public bool useSceneBallSpawn = false;
    private Vector3 sceneBallStartPosition;
    private Vector3 sceneBallStartScale;

    private Vector3 startPosition;
    private Quaternion startRotation;

    // --- Пенальти и Награды ---
    private float lastDistance = 1f;
    private bool wasSeeingBallLastStep = false;
    private int holdTicks = 0;

    // --- Action Rate Penalty (NVIDIA best practice) ---
    private float prevGas = 0f;
    private float prevSteering = 0f;
    private float prevCameraYaw = 0f;

    // --- Фаза 3: Слепой захват ---
    private bool wasCloseToBall = false;
    private float lastCloseAngle = 0f;
    private int blindApproachTicks = 0;
    private const int BLIND_APPROACH_MAX = 80;

    // --- Retry: Отъехать назад и попробовать снова ---
    private bool isRetrying = false;
    private int retryBackupTicks = 0;
    private int retryCount = 0;
    private const int MAX_RETRIES = 2;
    private const int RETRY_BACKUP_DURATION = 80;

    // --- Анти-застревание (Stuck Detection) ---
    private Vector3 lastPosition;
    private int stuckTimer = 0;

    // --- Training CSV telemetry (server-safe: one row per episode) ---
    private static StreamWriter trainingCsvWriter;
    private static string trainingCsvPath;
    private static int trainingCsvRows = 0;
    private static int parallelTrainingAgentCount = 0;
    private static int parallelEpisodeCount = 0;
    private static int parallelEpisodeSuccessCount = 0;
    private int episodeDecisionCount = 0;
    private int episodeBallSeenCount = 0;
    private int episodeReverseCount = 0;
    private float episodeGasAbsSum = 0f;
    private float episodeSteeringAbsSum = 0f;
    private float episodeMinBallDistance = 1f;
    private readonly float[] episodeRewardSums = new float[REWARD_COMPONENT_COUNT];
    private readonly int[] episodeRewardCounts = new int[REWARD_COMPONENT_COUNT];
    private int firstBallSeenStep = -1;
    private bool grabDiagnosticsRecorded = false;
    private bool episodeFinishRecorded = false;

    // --- Burst Dropout (YOLO теряет мяч пачками, не покадрово) ---
    private int burstDropoutRemaining = 0;

    // --- Одноразовый бонус за остановку ---
    private float lastCamDelta = 0f;

    // --- Клешня: трекинг физического состояния серво на реальном роботе ---
    private bool physicalServoCloseCmd = false;
    private int failedGrabTicks = 0;
    private const int FAILED_GRAB_THRESHOLD = 25;

    // --- Принудительная пауза после закрытия клешни (v15) ---
    private int gripperHoldTicks = 0;
    private const int GRIPPER_HOLD_DURATION = 25; // 25 тиков × 20мс = 500мс пауза

    private float lastVisualTargetTime = -1000f;
    private float lastVisualTargetX;
    private float lastVisualTargetDistance = 1f;
    private float visualGrabAlignedSeconds;
    private float visualGrabCooldownUntil;
    private bool visualGrabRequested;
    private bool physicalArmPrepared;
    [SerializeField] private bool physicalGripLatched;
    private float physicalArmReadyTime;
    [SerializeField] private bool physicalCloseSequenceActive;
    private float physicalCloseSequenceUntil;
    private float nextPhysicalCloseCommandTime;
    [SerializeField] private int physicalCloseTargetClassId = -1;
    [SerializeField] private int physicalGrabAttemptCount;
    private int lastVisualTargetClassId = -1;
    private string lastVisualTargetLabel = "";

    // --- v17: Таймер удержания клешни при потере ИК (анти-дребезг) ---
    private int holdWithoutIR = 0;
    private const int HOLD_WITHOUT_IR_MAX = 100; // 100 × 20мс = 2 сек удержания

    // --- v17: Клешня закрывается только если мяч был виден недавно ---
    private int lastBallSeenStep = -999;
    private float lastBallSeenDistance01 = 1f;
    private const int BALL_SEEN_WINDOW = 50; // 50 тиков = 1 сек

    // --- Camera Step Limit: реальное серво MAX_CAMERA_STEP = 15°/тик ---
    private const float MAX_CAMERA_STEP_NORMALIZED = 15f / 90f; // 15° из 90° диапазона

    public override void Initialize()
    {
        track = GetComponent<TrackController>();
        sensors = GetComponent<VirtualSensors>();
        rb = GetComponent<Rigidbody>();
        diagLogger = GetComponent<DiagnosticLogger>();
        gripperIrAutoClose = GetComponent<GripperIRAutoClose>();

        if (!isTraining && forceRealSensorsForInference && sensors != null)
        {
            sensors.EnableRealSensors();
        }

        if (!isTraining && realVision != null)
        {
            realVision.StartListener();
        }

        if (!isTraining &&
            ensureIndependentIrAutoClose &&
            gripperIrAutoClose == null)
        {
            gripperIrAutoClose = gameObject.AddComponent<GripperIRAutoClose>();
            Debug.Log(
                "[RobotBrain] Added GripperIRAutoClose for per-frame physical pickup safety.",
                this
            );
        }

        if (!isTraining &&
            ballOnlyAutonomousInference &&
            gripperIrAutoClose != null)
        {
            gripperIrAutoClose.gripPreset =
                GripperIRAutoClose.GripPreset.Ball;
        }

        if (!isTraining)
        {
            Debug.Log(
                $"[RobotBrain] Autonomous inference ready: " +
                $"target={(ballOnlyAutonomousInference ? "BALL class 0" : "configured target")}, " +
                $"realSensors={sensors != null && sensors.useRealSensors}, " +
                $"IR auto-close={gripperIrAutoClose != null}, " +
                $"ROS={(rosBridge != null ? "assigned" : "missing")}.",
                this
            );
        }

        startPosition = transform.position;
        startRotation = transform.rotation;

        if (!isTraining && autoEnableMovementForInference)
        {
            isMovementEnabled = true;
        }

        // --- Инициализация мяча для параллельных зон ---
        if (isTraining && ballPrefab != null)
        {
            // Создаем из префаба как потомка нашей тренировочной лаборатории
            spawnedBall = Instantiate(ballPrefab, transform.parent);
            spawnedBall.name = "TargetBall_Instance";
        }
        else
        {
            // Если префаб не задан, ищем локально среди соседей в иерархии
            spawnedBall = FindLocalBall();
        }

        if (spawnedBall != null)
        {
            sceneBallStartPosition = spawnedBall.transform.position;
            sceneBallStartScale = spawnedBall.transform.localScale;
        }

        BindVisionToLocalBall();

        if (isTraining)
        {
            RefreshParallelTrainingAgentCount();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            isMovementEnabled = !isMovementEnabled;
            Debug.Log("Движение " + (isMovementEnabled ? "разрешено!" : "запрещено!"));
            
            // v27: Автоматический сброс LSTM и одометрии при старте движения
            if (isMovementEnabled)
            {
                ResetInferenceEpisode();
            }
        }
        
        // Ручной сброс кнопкой R (полезно при тестах)
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetInferenceEpisode();
        }
    }

    public void ResetInferenceEpisode()
    {
        Debug.Log("[RobotBrain] СБРОС: Обнуление LSTM-памяти и одометрии для чистого старта");
        FinishEpisode(EpisodeEndReason.Other, false, lastDistance);
    }

    public override void OnEpisodeBegin()
    {
        // v27: Всегда сбрасываем позицию и ротацию виртуального робота (для тренировки и инференса!)
        // Это обнуляет displacementX и displacementZ
        transform.position = startPosition;
        transform.rotation = startRotation;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        ResetEpisodeDiagnostics();

        if (isTraining)
        {
            ResetTrainingEpisodeMetrics();

            // Рандомизация стартовой ротации — агент учится искать мяч в любом направлении
            float randomYaw = UnityEngine.Random.Range(-180f, 180f);
            transform.rotation = startRotation * Quaternion.Euler(0f, randomYaw, 0f);

            // --- Сброс мяча (Stage 3) ---
            if (useSceneBallSpawn) ResetBallToSceneSpawn();
            else ResetBall();
            BindVisionToLocalBall();
        }
        else if (resetBallNearRobotForInference && (sensors == null || !sensors.useRealSensors))
        {
            if (useSceneBallSpawn) ResetBallToSceneSpawn();
            else ResetBall(true);
            BindVisionToLocalBall();
        }

         // --- Domain Randomization (Физика) — расширенные диапазоны для Sim-to-Real ---
        if (isTraining && track != null)
        {
            track.moveSpeed = UnityEngine.Random.Range(0.3f, 0.7f);   // ±40% (реальный макс 0.5 м/с)
            track.turnSpeed = UnityEngine.Random.Range(80f, 160f);    // ±33%
            track.smoothing = UnityEngine.Random.Range(0.01f, 0.25f); // Разная "отзывчивость" моторов
        }
        if (isTraining && rb != null)
        {
            rb.mass = UnityEngine.Random.Range(1.0f, 4.0f); // Реальный ~2.5кг ± батарея/груз
        }

        // Сброс таймеров
        holdTicks = 0;
        stuckTimer = 0;
        lastPosition = transform.position;
        lastDistance = 1f; 
        wasSeeingBallLastStep = false;
        wasCloseToBall = false;
        lastCloseAngle = 0f;
        blindApproachTicks = 0;
        isRetrying = false;
        retryBackupTicks = 0;
        retryCount = 0;
        currentCameraYaw = 0f;
        physicalServoCloseCmd = false;
        failedGrabTicks = 0;
        gripperHoldTicks = 0;
        lastVisualTargetTime = -1000f;
        lastVisualTargetX = 0f;
        lastVisualTargetDistance = 1f;
        visualGrabAlignedSeconds = 0f;
        visualGrabCooldownUntil = 0f;
        visualGrabRequested = false;
        physicalArmPrepared = false;
        physicalGripLatched = false;
        physicalArmReadyTime = 0f;
        physicalCloseSequenceActive = false;
        physicalCloseSequenceUntil = 0f;
        nextPhysicalCloseCommandTime = 0f;
        physicalCloseTargetClassId = -1;
        physicalGrabAttemptCount = 0;
        gripperIrAutoClose?.ResetLatch();
        lastVisualTargetClassId = -1;
        lastVisualTargetLabel = "";
        visionApproachActive = false;
        visionApproachState = "idle";
        visionHeadingError = 0f;
        visionDriveCommand = 0f;
        visionSteeringCommand = 0f;
        holdWithoutIR = 0;
        lastBallSeenStep = -999;
        lastBallSeenDistance01 = 1f;
        prevGas = 0f;
        prevSteering = 0f;
        prevCameraYaw = 0f;
        burstDropoutRemaining = 0;
        if (cameraPivot != null) cameraPivot.localRotation = Quaternion.Euler(0, 0, 0);

        // --- Latency: рандомизация задержки каждый эпизод ---
        if (isTraining)
        {
            currentActionLatency = UnityEngine.Random.Range(minActionLatency, maxActionLatency + 1);
            actionBuffer.Clear();
            discreteActionBuffer.Clear();
            sensorBuffer.Clear();
            delayedSensors = new float[] { 1f, 0f, 0f, 0f };
            // Заполняем буфер "нулевыми" действиями
            for (int i = 0; i < currentActionLatency; i++)
            {
                actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
                discreteActionBuffer.Enqueue(new int[] { 0 });
            }
            for (int i = 0; i < sensorLatency; i++)
            {
                sensorBuffer.Enqueue(new float[] { 1f, 0f, 0f, 0f });
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // --- Domain Randomization (Шум сенсоров) ---
        float noiseUS = isTraining ? UnityEngine.Random.Range(-0.05f, 0.05f) : 0f;
        // РАЗДЕЛЬНЫЙ шум: YOLO angle точный (bbox center), distance шумный (bbox height)
        float noiseAmp = isTraining ? Academy.Instance.EnvironmentParameters.GetWithDefault("vision_noise", 0.02f) : 0f;
        float noiseVisAngle = isTraining ? UnityEngine.Random.Range(-noiseAmp, noiseAmp) : 0f;
        float noiseVisDist = isTraining ? UnityEngine.Random.Range(-noiseAmp * 3f, noiseAmp * 3f) : 0f; // Дистанция в 3× шумнее угла
        // BURST DROPOUT: YOLO теряет мяч пачками (3-8 кадров), не покадрово
        float dropoutRate = isTraining ? Unity.MLAgents.Academy.Instance.EnvironmentParameters.GetWithDefault("vision_dropout", 0f) : 0f;
        float bodyRotDropout = (rb != null && rb.angularVelocity.magnitude > 0.5f) ? 0.12f : 0f;
        float effectiveDropout = dropoutRate + (lastCamDelta > 0.3f ? 0.15f : 0f) + bodyRotDropout;
        // Пачечный dropout: если сработал — теряем на 3-8 кадров подряд
        if (burstDropoutRemaining > 0)
        {
            burstDropoutRemaining--;
        }
        else if (isTraining && UnityEngine.Random.value < effectiveDropout)
        {
            burstDropoutRemaining = UnityEngine.Random.Range(5, 16); // 5-15 кадров потери (реальный YOLO: до 48 шагов)
        }
        bool yoloDropout = burstDropoutRemaining > 0;

        // --- Latency: Задержка сенсоров (имитация запаздывания ROS-топиков) ---
        if (isTraining && sensorLatency > 0 && sensors != null)
        {
            // Текущие данные кладем в буфер
            sensorBuffer.Enqueue(new float[] {
                Mathf.Clamp01(sensors.ultrasonicDist + noiseUS),
                (float)sensors.leftIR,
                (float)sensors.rightIR,
                (float)sensors.gripperIR
            });
            // Достаем устаревшие данные
            if (sensorBuffer.Count > sensorLatency)
            {
                delayedSensors = sensorBuffer.Dequeue();
            }
            sensor.AddObservation(delayedSensors[0]);
            sensor.AddObservation((int)delayedSensors[1]);
            sensor.AddObservation((int)delayedSensors[2]);
            sensor.AddObservation((int)delayedSensors[3]);
        }
        else if (sensors != null)
        {
            sensor.AddObservation(Mathf.Clamp01(sensors.ultrasonicDist + noiseUS));
            sensor.AddObservation(sensors.leftIR);
            sensor.AddObservation(sensors.rightIR);
            sensor.AddObservation(sensors.gripperIR);
        }
        else
        {
            sensor.AddObservation(1f);
            sensor.AddObservation(0);
            sensor.AddObservation(0);
            sensor.AddObservation(0);
        }

        // --- Наблюдения Stage 4 (Зрение и память) ---
        if (realVision != null)
        {
            // ИСПРАВЛЕНО: Neural Network обучалась на том, что когда мяча нет, угол строго = 0f.
            // Передача lastKnownBallDirection в слот угла вызывала неадекватное поведение.
            bool ballVisible = realVision.seesBall;
            if (ballVisible)
            {
                lastBallSeenStep = StepCount;
                lastBallSeenDistance01 = Mathf.Clamp01(realVision.normalizedDistance);
            }
            sensor.AddObservation(ballVisible ? Mathf.Clamp(realVision.normalizedAngle, -1f, 1f) : 0f);
            sensor.AddObservation(ballVisible ? Mathf.Clamp01(realVision.normalizedDistance) : 1f);
            sensor.AddObservation(realVision.lastKnownBallDirection);
            sensor.AddObservation(ballVisible ? 1f : 0f);
            sensor.AddObservation(currentCameraYaw);
        }
        else if (simulatedYolo != null)
        {
            // SimulatedYoloCamera: YOLO-идентичные наблюдения через проекцию камеры
            bool ballVisible = simulatedYolo.seesBall && !yoloDropout;
            if (ballVisible)
            {
                lastBallSeenStep = StepCount;
                lastBallSeenDistance01 = Mathf.Clamp01(simulatedYolo.normalizedDistance);
            }
            
            sensor.AddObservation(ballVisible ? Mathf.Clamp(simulatedYolo.normalizedAngle + noiseVisAngle, -1f, 1f) : 0f);
            sensor.AddObservation(ballVisible ? Mathf.Clamp01(simulatedYolo.normalizedDistance + noiseVisDist) : 1f);
            sensor.AddObservation(simulatedYolo.lastKnownBallDirection);
            sensor.AddObservation(ballVisible ? 1f : 0f);
            sensor.AddObservation(currentCameraYaw);
        }
        else if (virtualCamera != null)
        {
            // Legacy fallback: VirtualCamera (raycast-based)
            bool ballVisible = virtualCamera.seesBall && !yoloDropout;
            if (ballVisible)
            {
                lastBallSeenStep = StepCount;
                lastBallSeenDistance01 = Mathf.Clamp01(virtualCamera.normalizedDistance);
            }
            
            sensor.AddObservation(ballVisible ? Mathf.Clamp(virtualCamera.normalizedAngle + noiseVisAngle, -1f, 1f) : 0f);
            sensor.AddObservation(ballVisible ? Mathf.Clamp01(virtualCamera.normalizedDistance + noiseVisDist) : 1f);
            sensor.AddObservation(virtualCamera.lastKnownBallDirection);
            sensor.AddObservation(ballVisible ? 1f : 0f);
            sensor.AddObservation(currentCameraYaw);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        if (gripper != null)
        {
            sensor.AddObservation(gripper.hasBall ? 1f : 0f);
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // === v16: EGOCENTRIC FEATURES (Self-Localization) ===
        // Дают агенту понимание "где я" и "куда двигаюсь" без полной карты местности.
        // Это позволяет LSTM строить внутреннюю модель пространства.

        // 1. Смещение от стартовой позиции (эгоцентрическое, нормализованное)
        Vector3 displacement = transform.position - startPosition;
        sensor.AddObservation(Mathf.Clamp(displacement.x / 3f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(displacement.z / 3f, -1f, 1f));

        // 2. Heading (направление взгляда робота, 0-1)
        float heading = transform.eulerAngles.y / 360f;
        sensor.AddObservation(heading);

        // 3. Собственная скорость (по сути одометрия)
        if (rb != null)
        {
            float speed = rb.linearVelocity.magnitude;
            sensor.AddObservation(Mathf.Clamp01(speed / 0.5f)); // 0=стоит, 1=макс
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // 4. Время с момента последнего видения мяча (0=только что видел, 1=давно не видел)
        float timeSinceSeen = 1f;
        if (simulatedYolo != null && simulatedYolo.seesBall)
            timeSinceSeen = 0f;
        else if (realVision != null && realVision.seesBall)
            timeSinceSeen = 0f;
        else
            timeSinceSeen = Mathf.Clamp01((float)blindApproachTicks / 100f);
        sensor.AddObservation(timeSinceSeen);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // --- "Ручник" (принудительный стоп) ---
        if (!isMovementEnabled)
        {
            if (track != null)
            {
                track.Move(0f, 0f);
                if (rosBridge != null)
                {
                    rosBridge.PublishCommand(0f, 0f);
                }
            }
            return;
        }

        float gas, steering, cameraYawInput;

        if (isTraining && currentActionLatency > 0)
        {
            // --- LATENCY: Кладем текущее действие нейросети в буфер ---
            float[] newContinuous = new float[] {
                actions.ContinuousActions[0],
                actions.ContinuousActions[1],
                actions.ContinuousActions.Length > 2 ? actions.ContinuousActions[2] : 0f
            };
            actionBuffer.Enqueue(newContinuous);

            // Достаем УСТАРЕВШЕЕ действие из головы очереди
            float[] delayed = actionBuffer.Dequeue();
            gas = delayed[0];
            steering = delayed[1];
            cameraYawInput = delayed[2];
        }
        else
        {
            // Без задержки (инференс на реальном роботе)
            gas = actions.ContinuousActions[0];
            steering = actions.ContinuousActions[1];
            cameraYawInput = actions.ContinuousActions.Length > 2 ? actions.ContinuousActions[2] : 0f;
        }

        ApplyInferenceVisionApproach(ref gas, ref steering, ref cameraYawInput);

        if (enableVerboseLogging)
        {
            Debug.Log($"[RobotBrain] Inputs -> BallSeen: {realVision?.seesBall}, Angle: {realVision?.normalizedAngle:F2}, Dist: {realVision?.normalizedDistance:F2}");
            Debug.Log($"[RobotBrain] Sensors -> UZ: {sensors?.ultrasonicDist:F2}, IR_L: {sensors?.leftIR}, IR_R: {sensors?.rightIR}, RealSensors: {sensors?.useRealSensors}");
            Debug.Log($"[RobotBrain] FINAL Output -> Gas: {gas:F2}, Steer: {steering:F2} (Latency: {currentActionLatency} steps)");
            // Диагностика задержек для сравнения sim vs real
            Debug.Log($"[RobotBrain] TIMING -> Time.time: {Time.time:F3}, deltaTime: {Time.deltaTime:F4}, fixedDelta: {Time.fixedDeltaTime:F4}, Step: {StepCount}");
        }

        // ============================================================
        // ФАЗА 0: МЯЧ СХВАЧЕН — ПРОВЕРЯЕМ САМЫМ ПЕРВЫМ!
        // Ничто не должно убивать эпизод, пока мяч в клешне.
        // ============================================================
        bool autoCloseLatched =
            gripperIrAutoClose != null && gripperIrAutoClose.GrabLatched;
        bool physicalBallHeld =
            !isTraining &&
            (physicalServoCloseCmd || autoCloseLatched) &&
            (physicalGripLatched ||
             autoCloseLatched ||
             (sensors != null && sensors.gripperIR == 1));
        bool unityBallHeld = gripper != null && gripper.hasBall;

        if (unityBallHeld || physicalBallHeld)
        {
            if (physicalBallHeld)
            {
                physicalGripLatched = true;
                physicalServoCloseCmd = true;
            }

            if (!isTraining && !wasHoldingBallLastStep)
            {
                successfulPickups++;
                wasHoldingBallLastStep = true;
                Debug.Log($"[InferenceStats] successfulPickups={successfulPickups}");
            }

            holdTicks++;
            stuckTimer = 0;
            AddTrackedReward(0.02f, RewardComponent.Grab);

            if (track != null) track.Move(0f, 0f);
            if (rosBridge != null && !isTraining)
            {
                rosBridge.PublishCommand(0f, 0f);
                if (!physicalServoCloseCmd)
                {
                    rosBridge.CloseGripperForTargetClass(GetActiveTargetClassId());
                    physicalServoCloseCmd = true;
                    physicalGripLatched = true;
                }
            }

            if (isTraining && holdTicks >= 50)
            {
                AddTrackedReward(5.0f, RewardComponent.Grab);
                Unity.MLAgents.Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 1.0f);
                FinishEpisode(EpisodeEndReason.Success, true, 0f, "grab_success");
                return;
            }
            return;
        }
        holdTicks = 0;
        wasHoldingBallLastStep = false;

        // === АНТИ-ЗАСТРЕВАНИЕ И ЛИМИТ ЭПИЗОДА ===
        int episodeLimit = Mathf.RoundToInt(
            Academy.Instance.EnvironmentParameters.GetWithDefault("episode_length", 800f)
        );
        if (isTraining && StepCount >= episodeLimit)
        {
            AddTrackedReward(-0.05f, RewardComponent.TimePenalty); // Лёгкий штраф — не наказываем за сложное расположение мяча
            Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 0.0f);
            FinishEpisode(EpisodeEndReason.Timeout, false, lastDistance, "timeout");
            return;
        }

        if (Mathf.Abs(gas) > 0.1f || Mathf.Abs(steering) > 0.1f)
        {
            stuckTimer++;
            if (stuckTimer >= 200)
            {
                float distanceTravelled = Vector3.Distance(transform.position, lastPosition);
                if (distanceTravelled < 0.5f)
                {
                    AddTrackedReward(-0.5f, RewardComponent.CollisionPenalty);
                    if (isTraining) Unity.MLAgents.Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 0.0f);
                    FinishEpisode(EpisodeEndReason.Stuck, false, lastDistance, "stuck");
                    return;
                }
                stuckTimer = 0;
                lastPosition = transform.position;
            }
        }
        else
        {
            stuckTimer = 0;
            lastPosition = transform.position;
        }

        // --- Camera Step Limit: реальное серво двигается макс 15°/тик ---
        float targetCamYaw = Mathf.Clamp(cameraYawInput, -1f, 1f);
        float camDelta = targetCamYaw - currentCameraYaw;
        if (Mathf.Abs(camDelta) > MAX_CAMERA_STEP_NORMALIZED)
        {
            targetCamYaw = currentCameraYaw + Mathf.Sign(camDelta) * MAX_CAMERA_STEP_NORMALIZED;
        }
        lastCamDelta = Mathf.Abs(targetCamYaw - currentCameraYaw);
        currentCameraYaw = targetCamYaw;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = Quaternion.Euler(0f, currentCameraYaw * 90f, 0f);
        }

        // === ПРИНУДИТЕЛЬНАЯ ОСТАНОВКА ПРИ ЗАКРЫТИИ КЛЕШНИ (v15) ===
        // Когда клешня закрывается, робот должен стоять и ждать подтверждения gripperIR.
        // Без этого робот проезжает мимо мяча после закрытия клешни.
        if (gripperHoldTicks > 0)
        {
            gas = 0f;
            steering = 0f;
            gripperHoldTicks--;
        }

        LogInferenceActionIfNeeded(gas, steering);

        if (track != null)
        {
            // Убрали EndEpisode при столкновении по сонару, так как это учило агента вообще не ехать к стенам
            // Штраф за физические столкновения обрабатывается в СЕНСОРНЫЕ ШТРАФЫ

            track.Move(gas, steering);

            if (rosBridge != null && !isTraining)
            {
                rosBridge.PublishCommand(gas, steering);
                rosBridge.PublishCameraCmd(currentCameraYaw);
            }
        }

        // --- Логика клешни ---
        // Реальный ИК в захвате имеет приоритет: после аппаратного debounce он
        // закрывает клешню даже если объект уже исчез из кадра камеры.
        bool gripperSensorActive = sensors != null && sensors.gripperIR == 1;

        // В симуляции мяч теряет коллайдер при захвате, ИК перестает его видеть (bug).
        // Симулируем, что ИК все еще видит мяч, если он уже в клешне.
        if (sensors != null && !sensors.useRealSensors && gripper != null && gripper.hasBall)
        {
            gripperSensorActive = true;
        }

        if (gripper != null || (!isTraining && rosBridge != null))
        {
            bool unityHasBall = gripper != null && gripper.hasBall;
            bool ballRecentlySeen = (StepCount - lastBallSeenStep) < BALL_SEEN_WINDOW;
            bool targetCloseEnough =
                !requireCloseVisualTargetForInferenceGrab ||
                lastBallSeenDistance01 <= maxInferenceGrabDistance01 ||
                (visualGrabRequested &&
                 lastBallSeenDistance01 <= blindGrabStartDistance01);
            bool allowGrip =
                isTraining ||
                gripperSensorActive ||
                (ballRecentlySeen && targetCloseEnough);
            bool requestGrip = gripperSensorActive || visualGrabRequested;

            if (!unityHasBall &&
                requestGrip &&
                allowGrip &&
                (isTraining || !physicalServoCloseCmd))
            {
                gripperHoldTicks = GRIPPER_HOLD_DURATION;
                visualGrabRequested = false;
                visualGrabCooldownUntil = Time.unscaledTime + 1.5f;
                if (isTraining)
                {
                    CloseUnityGripperForActiveTarget();
                }
                else
                {
                    BeginPhysicalGrabAttempt(gripperSensorActive);
                }
            }
            else if (!unityHasBall && gripperSensorActive && !allowGrip)
            {
                // ИК сработал, но мяч не был виден — ложное срабатывание, игнорируем
                if (rosBridge != null && !isTraining && physicalServoCloseCmd)
                {
                    rosBridge.PublishGripperCmd(4); // Открыть клешню назад
                    physicalServoCloseCmd = false;
                    physicalGripLatched = false;
                    Debug.Log(
                        $"[RobotBrain] Захват отменен: targetAge={StepCount - lastBallSeenStep}, " +
                        $"visionDistance={lastBallSeenDistance01:F2}");
                }
            }
            else if (unityHasBall && !gripperSensorActive)
            {
                // v17: НЕ разжимаем мгновенно! ИК может дребезжать при сдвиге мяча.
                // Ждём 2 секунды без ИК-сигнала перед разжатием.
                holdWithoutIR++;
                if (holdWithoutIR >= HOLD_WITHOUT_IR_MAX)
                {
                    gripper.OpenGripper();
                    holdWithoutIR = 0;
                    if (rosBridge != null && !isTraining)
                    {
                        rosBridge.PublishGripperCmd(4);
                        physicalServoCloseCmd = false;
                        physicalGripLatched = false;
                        failedGrabTicks = 0;
                    }
                }
            }
            else if (unityHasBall && gripperSensorActive)
            {
                // ИК снова видит мяч — сбрасываем таймер разжатия
                holdWithoutIR = 0;
            }
            else if (!isTraining && !unityHasBall && !gripperSensorActive && physicalServoCloseCmd)
            {
                failedGrabTicks++;
                if (failedGrabTicks >= FAILED_GRAB_THRESHOLD)
                {
                    rosBridge?.PublishGripperCmd(4);
                    physicalServoCloseCmd = false;
                    physicalGripLatched = false;
                    failedGrabTicks = 0;
                }
            }
            else if (!gripperSensorActive)
            {
                failedGrabTicks = 0;
            }
        }

        // (ФАЗА 0 перенесена выше — до проверок EndEpisode)

        // === REWARD #4: SENSOR PROXIMITY (заменяет OnCollisionEnter) ===
        // Штраф по ДАТЧИКАМ, а не по коллизиям — на реальном роботе коллизий нет
        if (isTraining && sensors != null)
        {
            // УЗ: градиентный штраф (чем ближе стена — тем больше)
            if (sensors.ultrasonicDist < 0.12f) // < 60см при maxRange 5м
            {
                float sonarProx = 1f - (sensors.ultrasonicDist / 0.12f);
                AddTrackedReward(-0.03f * sonarProx, RewardComponent.CollisionPenalty);
            }

            // ИК: бинарный штраф (датчики бинарные — 0 или 1)
            if (sensors.leftIR == 1 || sensors.rightIR == 1)
            {
                AddTrackedReward(-0.01f, RewardComponent.CollisionPenalty);
            }
        }

        // === REWARD #5: ACTION RATE PENALTY (NVIDIA Isaac Lab стандарт) ===
        if (isTraining)
        {
            float actionRate = Mathf.Pow(gas - prevGas, 2)
                             + Mathf.Pow(steering - prevSteering, 2)
                             + Mathf.Pow(cameraYawInput - prevCameraYaw, 2);
            AddTrackedReward(-0.05f * actionRate, RewardComponent.ActionPenalty);

            // === REWARD #6: MILD REVERSE PENALTY (v15) ===
            // v12 удалил агрессивный -0.02 (был воркараунд для бага мотора).
            // Мотор исправлен, но без ЛЮБОГО штрафа reverse=41% — слишком много.
            // Мягкий -0.005 + исключения для retry и стен.
            if (gas < -0.1f && !isRetrying)
            {
                bool nearWall = sensors != null && sensors.ultrasonicDist < 0.12f;
                bool nearSideWall = sensors != null && (sensors.leftIR == 1 || sensors.rightIR == 1);
                if (!nearWall && !nearSideWall)
                {
                    AddTrackedReward(-0.005f, RewardComponent.ReversePenalty);
                }
            }
        }
        prevGas = gas;
        prevSteering = steering;
        prevCameraYaw = cameraYawInput;

        // === ДИАГНОСТИКА (TensorBoard) ===
        if (isTraining)
        {
            Academy.Instance.StatsRecorder.Add("Custom/Gas", gas);
            Academy.Instance.StatsRecorder.Add("Custom/IsReverse", gas < -0.1f ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("Custom/BlindTicks", blindApproachTicks);
        }

        // === СЧИТЫВАЕМ ЗРЕНИЕ ===
        bool hasSeenBall = false;
        float currentAngle = 0f;
        float currentDist = 0f;

        if (realVision != null && realVision.seesBall)
        {
            hasSeenBall = true;
            currentAngle = realVision.normalizedAngle;
            currentDist = realVision.normalizedDistance;
        }
        else if (simulatedYolo != null && simulatedYolo.seesBall)
        {
            hasSeenBall = true;
            currentAngle = simulatedYolo.normalizedAngle;
            currentDist = simulatedYolo.normalizedDistance;
        }
        else if (virtualCamera != null && virtualCamera.seesBall)
        {
            hasSeenBall = true;
            currentAngle = virtualCamera.normalizedAngle;
            currentDist = virtualCamera.normalizedDistance;
        }

        RecordTrainingDecisionMetrics(gas, steering, hasSeenBall, currentDist);

        // === ПРОВЕРКА ИК-ДАТЧИКА КЛЕШНИ ===
        bool gripperSeesBall = sensors != null && sensors.gripperIR == 1;

        if (gripperSeesBall)
        {
            isRetrying = false;
            // Убраны отдельные бонусы/штрафы — action rate penalty уже учит плавности
        }

        // === ФАЗА RETRY: Отъезд назад после промаха ===
        // КРИТИЧЕСКИЙ ФИКС: Если мы сдаем назад, и вдруг мяч появился снова - НЕМЕДЛЕННО обрываем Retry!
        if (isRetrying && hasSeenBall)
        {
            isRetrying = false;
            wasCloseToBall = false;
            retryBackupTicks = 0;
            blindApproachTicks = 0;
        }

        if (isRetrying)
        {
            retryBackupTicks++;

            // Retry — аварийный режим. Без отдельных штрафов.
            // Action rate penalty автоматически штрафует резкие переключения.

            if (retryBackupTicks >= RETRY_BACKUP_DURATION)
            {
                isRetrying = false;
                wasCloseToBall = false;
                retryBackupTicks = 0;
                blindApproachTicks = 0;
                lastDistance = 1f;
            }
            return;
        }

        if (hasSeenBall)
        {
            RecordFirstBallSeen();

            // Убираем спайк-награду: если мяч только появился, дельта-награды не будет
            if (!wasSeeingBallLastStep)
            {
                lastDistance = currentDist;
            }

            // Мяч видим камерой — retry-сброс при повторном обнаружении
            blindApproachTicks = 0;
            isRetrying = false;

            // === REWARD #1: DISTANCE DELTA (proximity-scaled) ===
            // Чем ближе мяч — тем ВАЖНЕЕ точность приближения (множитель растёт).
            // dist=0.8 → 2.8x, dist=0.3 → 4.8x, dist=0.1 → 5.6x
            if (wasSeeingBallLastStep)
            {
                float distanceDelta = lastDistance - currentDist;
                if (Mathf.Abs(distanceDelta) < 0.5f) // Фильтр спайков
                {
                    float proximityMultiplier = 2.0f + 4.0f * (1.0f - Mathf.Clamp01(currentDist));
                    AddTrackedReward(distanceDelta * proximityMultiplier, RewardComponent.Approach);
                }
            }

            // === REWARD #7: PROXIMITY SLOW-DOWN BONUS (v15, усилен) ===
            if (currentDist < 0.3f && gas > 0.01f && gas < 0.3f)
            {
                AddTrackedReward(0.005f, RewardComponent.Approach);
            }

            // === REWARD #8: SPEED PENALTY NEAR BALL (v26) ===
            // Штраф за высокую скорость вблизи мяча — не таранить!
            if (currentDist < 0.25f && Mathf.Abs(gas) > 0.4f)
            {
                AddTrackedReward(-0.01f, RewardComponent.Approach);
            }

            // === REWARD #9: ALIGNMENT BONUS (v26) ===
            // Бонус за центрированный мяч при близком подъезде.
            // Учит робота выравниваться перед захватом.
            if (currentDist < 0.4f && Mathf.Abs(currentAngle) < 0.15f)
            {
                AddTrackedReward(0.005f, RewardComponent.Alignment);
            }

            // Обновляем фазу для слепого подъезда
            if (currentDist <= 0.35f)
            {
                wasCloseToBall = true;
                lastCloseAngle = currentAngle;
            }
            else
            {
                wasCloseToBall = false;
            }

            lastDistance = currentDist;
            wasSeeingBallLastStep = true;
        }
        else
        {
            // === МЯЧ НЕ ВИДЕН ===

            if (wasCloseToBall && !gripperSeesBall)
            {
                // =======================================
                // ФАЗА 3: МЯЧ ПРОПАЛ (был рядом)
                // Мяч в слепой зоне камеры (под роботом/в клешне)
                // БЕЗ этого бонуса агент едет назад ("safe" action)
                // =======================================
                blindApproachTicks++;

                // Бонус за медленное движение вперёд (ползи к мячу, не отъезжай)
                if (gas > 0.01f && gas < 0.3f)
                {
                    AddTrackedReward(0.003f, RewardComponent.BlindApproach);
                }

                if (blindApproachTicks >= BLIND_APPROACH_MAX)
                {
                    if (retryCount < MAX_RETRIES)
                    {
                        isRetrying = true;
                        retryBackupTicks = 0;
                        retryCount++;
                        blindApproachTicks = 0;
                    }
                    else
                    {
                        wasCloseToBall = false;
                        blindApproachTicks = 0;
                    }
                }
            }
            // Фаза поиска: без отдельных наград/штрафов
            // Distance delta + action rate penalty достаточно

            lastDistance = 1f;
            wasSeeingBallLastStep = false;
        }

        // === CSV-ЛОГГЕР (для диагностики sim vs real) ===
        if (diagLogger != null)
        {
            // v15: Добавлен simulatedYolo в логгер (раньше BallSeen=0 в 100% sim-строк!)
            bool bs = (realVision != null && realVision.seesBall) 
                   || (simulatedYolo != null && simulatedYolo.seesBall) 
                   || (virtualCamera != null && virtualCamera.seesBall);
            float ba = realVision?.seesBall == true ? realVision.normalizedAngle 
                     : (simulatedYolo?.seesBall == true ? simulatedYolo.normalizedAngle 
                     : (virtualCamera?.seesBall == true ? virtualCamera.normalizedAngle : 0f));
            float bd = realVision?.seesBall == true ? realVision.normalizedDistance 
                     : (simulatedYolo?.seesBall == true ? simulatedYolo.normalizedDistance 
                     : (virtualCamera?.seesBall == true ? virtualCamera.normalizedDistance : 0f));
            
            // v18: Расширенные данные для диагностики
            bool ballRecent = (StepCount - lastBallSeenStep) < BALL_SEEN_WINDOW;
            Vector3 disp = transform.position - startPosition;
            float spd = rb != null ? rb.linearVelocity.magnitude : 0f;
            float hdg = transform.eulerAngles.y / 360f;
            
            diagLogger.LogStep(StepCount, bs, ba, bd,
                sensors?.ultrasonicDist ?? 1f, sensors?.leftIR ?? 0, sensors?.rightIR ?? 0, sensors?.gripperIR ?? 0,
                currentCameraYaw, gas, steering, cameraYawInput,
                gripper != null && gripper.hasBall, holdTicks, isRetrying, wasCloseToBall,
                blindApproachTicks, ballRecent, holdWithoutIR,
                Mathf.Clamp(disp.x / 3f, -1f, 1f), Mathf.Clamp(disp.z / 3f, -1f, 1f), hdg, spd,
                transform.position.x, transform.position.z,
                realVision != null ? realVision.yoloConfidence : 0f,
                realVision != null ? realVision.bboxWidth : 0f,
                realVision != null ? realVision.bboxHeight : 0f,
                sensors != null ? sensors.realPwmLeft : 0f,
                sensors != null ? sensors.realPwmRight : 0f);
        }
    }

    private void ApplyInferenceVisionApproach(
        ref float gas,
        ref float steering,
        ref float cameraYawInput)
    {
        visionApproachActive = false;
        visionApproachState = "policy";
        visionHeadingError = 0f;
        visionDriveCommand = gas;
        visionSteeringCommand = steering;
        visualGrabRequested = false;

        if (isTraining || !enableVisionApproachOverride || realVision == null)
        {
            return;
        }

        float now = Time.unscaledTime;
        float fixedDelta = Mathf.Max(0.001f, Time.fixedDeltaTime);
        bool gripperSensorActive = sensors != null && sensors.gripperIR == 1;

        if (MaintainPhysicalCloseSequence(now, gripperSensorActive))
        {
            visionApproachActive = true;
            gas = 0f;
            steering = 0f;
            visionApproachState = gripperSensorActive
                ? "closing claw: IR confirmed"
                : "closing claw: visual trigger";
            visionDriveCommand = 0f;
            visionSteeringCommand = 0f;
            return;
        }

        bool targetVisible = TryGetInferenceTarget(
            out float detectedX,
            out float detectedDistance,
            out int detectedClassId,
            out string detectedLabel
        );

        if (targetVisible)
        {
            visionApproachActive = true;
            lastVisualTargetTime = now;
            lastVisualTargetX = Mathf.Clamp(detectedX, -1f, 1f);
            lastVisualTargetDistance = Mathf.Clamp01(detectedDistance);
            lastVisualTargetClassId = detectedClassId;
            lastVisualTargetLabel = detectedLabel;
            lastBallSeenStep = StepCount;
            lastBallSeenDistance01 = lastVisualTargetDistance;

            if (preparePhysicalGripperOnTarget &&
                !physicalArmPrepared &&
                rosBridge != null)
            {
                rosBridge.PublishGripperCmd(1);
                physicalArmPrepared = true;
                physicalServoCloseCmd = false;
                physicalArmReadyTime = now + armPrepareWaitSeconds;
                Debug.Log(
                    $"[RobotBrain] Подготовка клешни для цели {detectedLabel}",
                    this
                );
            }

            if (physicalArmPrepared && now < physicalArmReadyTime)
            {
                gas = 0f;
                steering = 0f;
                cameraYawInput = Mathf.MoveTowards(
                    currentCameraYaw,
                    0f,
                    0.6f * fixedDelta
                );
                visionApproachState = "prepare arm and open claw";
                visionDriveCommand = 0f;
                visionSteeringCommand = 0f;
                return;
            }

            // Steer the chassis toward the target's world direction, not merely
            // toward the center of a camera that may itself be panned sideways.
            float headingError = Mathf.Clamp(
                currentCameraYaw + lastVisualTargetX * imageErrorToHeading,
                -1f,
                1f
            );
            visionHeadingError = headingError;

            float steeringDirection = invertVisionSteering ? 1f : -1f;
            float desiredAngularSpeed = Mathf.Clamp(
                steeringDirection * headingError * approachSteeringGainRad,
                -approachMaxAngularSpeedRad,
                approachMaxAngularSpeedRad
            );
            steering = NormalizeAngularSpeed(desiredAngularSpeed);

            // As the body turns, bring the camera back to its calibrated center.
            cameraYawInput = Mathf.MoveTowards(
                currentCameraYaw,
                0f,
                0.6f * fixedDelta
            );

            bool bodyAligned = Mathf.Abs(headingError) <= rotateInPlaceHeadingError;
            float distanceBlend = Mathf.InverseLerp(
                visualGrabDistance01,
                1f,
                lastVisualTargetDistance
            );
            float desiredSpeed = Mathf.Lerp(
                approachCreepSpeedMps,
                approachCruiseSpeedMps,
                distanceBlend
            );

            if (!bodyAligned)
            {
                gas = 0f;
                visionApproachState = $"turn to {detectedLabel}";
            }
            else
            {
                float alignmentScale = Mathf.Clamp01(
                    1f - Mathf.Abs(headingError) / Mathf.Max(0.01f, rotateInPlaceHeadingError)
                );
                desiredSpeed *= Mathf.Lerp(0.35f, 1f, alignmentScale);
                gas = NormalizeLinearSpeed(desiredSpeed);
                visionApproachState =
                    lastVisualTargetDistance <= approachSlowDistance01
                        ? $"creep to {detectedLabel}"
                        : $"approach {detectedLabel}";
            }

            bool centeredForGrab =
                Mathf.Abs(lastVisualTargetX) <= visualCenterTolerance &&
                Mathf.Abs(headingError) <= visualCenterTolerance;

            if (gripperSensorActive)
            {
                gas = 0f;
                steering = 0f;
                visualGrabRequested = true;
                visualGrabAlignedSeconds = 0f;
                visionApproachState = "gripper IR: grab";
            }
            else if (lastVisualTargetDistance <= visualGrabDistance01 && centeredForGrab)
            {
                visualGrabAlignedSeconds += fixedDelta;
                gas = NormalizeLinearSpeed(approachCreepSpeedMps);
                visionApproachState = "final creep";

                if (allowVisionOnlyGrab &&
                    (forceGrabAfterVisualApproach ||
                     !NeedsGripperIrForActiveTarget()) &&
                    visualGrabAlignedSeconds >= visualGrabConfirmSeconds &&
                    now >= visualGrabCooldownUntil)
                {
                    gas = 0f;
                    steering = 0f;
                    visualGrabRequested = true;
                    visionApproachState = "vision confirmed: grab";
                }
            }
            else
            {
                visualGrabAlignedSeconds = 0f;
            }
        }
        else if (
            now - lastVisualTargetTime <= GetBlindApproachSeconds() &&
            lastVisualTargetDistance <= blindGrabStartDistance01 &&
            Mathf.Abs(lastVisualTargetX) <= visualCenterTolerance * 1.5f)
        {
            // The target normally disappears below the camera just before it
            // reaches the gripper. Continue straight briefly instead of stopping.
            visionApproachActive = true;
            float headingError = Mathf.Clamp(
                currentCameraYaw + lastVisualTargetX * imageErrorToHeading,
                -1f,
                1f
            );
            visionHeadingError = headingError;

            float steeringDirection = invertVisionSteering ? 1f : -1f;
            float desiredAngularSpeed = Mathf.Clamp(
                steeringDirection * headingError * approachSteeringGainRad,
                -approachMaxAngularSpeedRad * 0.5f,
                approachMaxAngularSpeedRad * 0.5f
            );
            gas = NormalizeLinearSpeed(approachCreepSpeedMps);
            steering = NormalizeAngularSpeed(desiredAngularSpeed);
            cameraYawInput = Mathf.MoveTowards(
                currentCameraYaw,
                0f,
                0.6f * fixedDelta
            );
            visionApproachState = "blind final creep";

            if (gripperSensorActive)
            {
                gas = 0f;
                steering = 0f;
                visualGrabRequested = true;
                visionApproachState = "blind IR: grab";
            }
            else if (
                allowVisionOnlyGrab &&
                (forceGrabAfterVisualApproach ||
                 !NeedsGripperIrForActiveTarget()) &&
                now - lastVisualTargetTime >= blindGrabDelaySeconds &&
                now >= visualGrabCooldownUntil)
            {
                gas = 0f;
                steering = 0f;
                visualGrabRequested = true;
                visionApproachState = "blind close attempt";
            }
        }
        else
        {
            visualGrabAlignedSeconds = 0f;
        }

        visionDriveCommand = gas;
        visionSteeringCommand = steering;
    }

    private bool MaintainPhysicalCloseSequence(float now, bool gripperSensorActive)
    {
        if (!physicalCloseSequenceActive)
        {
            return false;
        }

        rosBridge?.PublishStop();

        if (now >= nextPhysicalCloseCommandTime)
        {
            rosBridge?.CloseGripperForTargetClass(physicalCloseTargetClassId);
            nextPhysicalCloseCommandTime =
                now + Mathf.Max(0.05f, physicalCloseRepeatInterval);
        }

        if (now >= physicalCloseSequenceUntil)
        {
            physicalCloseSequenceActive = false;
            Debug.Log(
                $"[RobotBrain] Клешня закрыта и удерживается: " +
                $"target={lastVisualTargetLabel}, class={physicalCloseTargetClassId}, " +
                $"IR={gripperSensorActive}, attempts={physicalGrabAttemptCount}.",
                this
            );
        }

        return true;
    }

    private void BeginPhysicalGrabAttempt(bool gripperSensorActive)
    {
        int targetClassId = GetActiveTargetClassId();
        float now = Time.unscaledTime;

        CloseUnityGripperForActiveTarget();
        rosBridge?.PublishStop();
        rosBridge?.CloseGripperForTargetClass(targetClassId);

        physicalServoCloseCmd = true;
        // The physical servo keeps its commanded angle even if the IR sensor
        // misses the object. Keep the drive stopped and never auto-open here.
        physicalGripLatched = true;
        physicalCloseTargetClassId = targetClassId;
        physicalCloseSequenceActive = true;
        physicalCloseSequenceUntil =
            now + Mathf.Max(0.1f, physicalCloseRepeatSeconds);
        nextPhysicalCloseCommandTime =
            now + Mathf.Max(0.05f, physicalCloseRepeatInterval);
        physicalGrabAttemptCount++;
        failedGrabTicks = 0;

        Debug.Log(
            $"[RobotBrain] НАЧАТ ЗАХВАТ: target={lastVisualTargetLabel}, " +
            $"class={targetClassId}, IR={gripperSensorActive}, " +
            $"S4={(targetClassId == 1 ? "cube" : "ball")} preset.",
            this
        );
    }

    private bool TryGetInferenceTarget(
        out float normalizedX,
        out float normalizedDistance,
        out int classId,
        out string label)
    {
        normalizedX = 0f;
        normalizedDistance = 1f;
        classId = -1;
        label = "";

        if (realVision == null)
        {
            return false;
        }

        int requestedClassId =
            !isTraining && ballOnlyAutonomousInference
                ? 0
                : approachBallAndCube
                    ? -1
                    : inferenceTargetClassId;
        if (!realVision.TryGetTarget(
                requestedClassId,
                out normalizedX,
                out normalizedDistance,
                out _,
                out label))
        {
            return false;
        }

        classId = requestedClassId >= 0
            ? requestedClassId
            : realVision.TargetClassId;
        if (string.IsNullOrWhiteSpace(label))
        {
            label = classId == 1 ? "cube" : classId == 0 ? "ball" : "target";
        }
        return true;
    }

    private int GetActiveTargetClassId()
    {
        if (!isTraining && ballOnlyAutonomousInference)
        {
            return 0;
        }

        if (lastVisualTargetClassId >= 0)
        {
            return lastVisualTargetClassId;
        }

        if (inferenceTargetClassId >= 0)
        {
            return inferenceTargetClassId;
        }

        return realVision != null ? realVision.TargetClassId : 0;
    }

    private bool NeedsGripperIrForActiveTarget()
    {
        int targetClassId = GetActiveTargetClassId();
        return (targetClassId == 1 && requireGripperIrForCube) ||
               (targetClassId == 0 && requireGripperIrForBall);
    }

    private float GetBlindApproachSeconds()
    {
        int targetClassId = GetActiveTargetClassId();
        if (targetClassId == 1)
        {
            return Mathf.Max(blindApproachSeconds, cubeBlindApproachSeconds);
        }
        if (targetClassId == 0)
        {
            return Mathf.Max(blindApproachSeconds, ballBlindApproachSeconds);
        }
        return blindApproachSeconds;
    }

    private void CloseUnityGripperForActiveTarget()
    {
        if (gripper == null)
        {
            return;
        }

        if (!isTraining && GetActiveTargetClassId() == 1)
        {
            gripper.SetJawClosure01(unityCubeJawClosure, true);
            return;
        }

        gripper.CloseGripper();
    }

    private float NormalizeLinearSpeed(float metersPerSecond)
    {
        float maximum = rosBridge != null
            ? Mathf.Max(0.01f, rosBridge.maxLinearSpeed)
            : track != null
                ? Mathf.Max(0.01f, track.maxLinearCmd)
                : 0.5f;
        return Mathf.Clamp(metersPerSecond / maximum, -1f, 1f);
    }

    private float NormalizeAngularSpeed(float radiansPerSecond)
    {
        float maximum = rosBridge != null
            ? Mathf.Max(0.01f, rosBridge.maxAngularSpeed)
            : 1f;
        return Mathf.Clamp(radiansPerSecond / maximum, -1f, 1f);
    }

    private GameObject FindLocalBall()
    {
        if (transform.parent != null)
        {
            Transform[] candidates = transform.parent.GetComponentsInChildren<Transform>(true);
            foreach (Transform t in candidates)
            {
                if (t != null && t.CompareTag("TargetBall")) return t.gameObject;
            }
        }
        return null;
    }

    private void BindVisionToLocalBall()
    {
        if (spawnedBall == null)
        {
            spawnedBall = FindLocalBall();
        }

        if (spawnedBall == null)
        {
            Debug.LogWarning($"[RobotBrain] Локальный TargetBall не найден для арены {transform.parent?.name ?? name}");
            return;
        }

        if (virtualCamera != null)
        {
            virtualCamera.targetBall = spawnedBall.transform;
        }

        if (simulatedYolo != null)
        {
            simulatedYolo.targetBall = spawnedBall.transform;
        }
    }

    private void ResetEpisodeDiagnostics()
    {
        for (int i = 0; i < REWARD_COMPONENT_COUNT; i++)
        {
            episodeRewardSums[i] = 0f;
            episodeRewardCounts[i] = 0;
        }

        firstBallSeenStep = -1;
        grabDiagnosticsRecorded = false;
        episodeFinishRecorded = false;
    }

    private void AddTrackedReward(float value, RewardComponent component)
    {
        AddReward(value);

        int index = (int)component;
        episodeRewardSums[index] += value;
        episodeRewardCounts[index]++;
    }

    public void AddTrackedBlindPenalty(float value)
    {
        AddTrackedReward(value, RewardComponent.BlindPenalty);
    }

    private void RecordFirstBallSeen()
    {
        if (firstBallSeenStep < 0)
        {
            firstBallSeenStep = StepCount;
        }
    }

    private void RecordSuccessfulGrabDiagnostics()
    {
        if (grabDiagnosticsRecorded) return;

        grabDiagnosticsRecorded = true;
        bool hadPriorSeen = firstBallSeenStep >= 0;
        Academy.Instance.StatsRecorder.Add("Custom/GrabWithoutPriorSeen", hadPriorSeen ? 0f : 1f);

        if (hadPriorSeen)
        {
            Academy.Instance.StatsRecorder.Add("Custom/StepsFromFirstSeenToGrab", StepCount - firstBallSeenStep);
        }
    }

    private void FinishEpisode(EpisodeEndReason endReason, bool success, float finalDistance, string trainingReason = null)
    {
        if (episodeFinishRecorded) return;
        episodeFinishRecorded = true;

        if (success)
        {
            RecordSuccessfulGrabDiagnostics();
        }

        FinishTrainingEpisode(trainingReason ?? GetEpisodeReasonName(endReason), success, finalDistance);
        RecordEpisodeEndDiagnostics(endReason);
        EndEpisode();
    }

    private void RecordEpisodeEndDiagnostics(EpisodeEndReason endReason)
    {
        float totalTracked = 0f;
        float perStepDivisor = Mathf.Max(1, StepCount);

        for (int i = 0; i < REWARD_COMPONENT_COUNT; i++)
        {
            float sum = episodeRewardSums[i];
            int count = episodeRewardCounts[i];
            totalTracked += sum;

            string componentName = RewardComponentNames[i];
            Academy.Instance.StatsRecorder.Add("Reward/" + componentName, sum);
            Academy.Instance.StatsRecorder.Add("RewardCount/" + componentName, count);
            Academy.Instance.StatsRecorder.Add("RewardPerStep/" + componentName, sum / perStepDivisor);
        }

        float cumulativeReward = GetCumulativeReward();
        float trackingError = cumulativeReward - totalTracked;

        Academy.Instance.StatsRecorder.Add("Reward/TotalTracked", totalTracked);
        Academy.Instance.StatsRecorder.Add("Reward/AgentCumulative", cumulativeReward);
        Academy.Instance.StatsRecorder.Add("Reward/TrackingError", trackingError);

        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Success", endReason == EpisodeEndReason.Success ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Timeout", endReason == EpisodeEndReason.Timeout ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Boundary", endReason == EpisodeEndReason.Boundary ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Collision", endReason == EpisodeEndReason.Collision ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Stuck", endReason == EpisodeEndReason.Stuck ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/Other", endReason == EpisodeEndReason.Other ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/StepCount", StepCount);
        Academy.Instance.StatsRecorder.Add("EpisodeEnd/CumulativeReward", cumulativeReward);

        if (debugRewardTracking && Mathf.Abs(trackingError) > 0.001f)
        {
            Debug.LogWarning($"[RobotBrain] Reward tracking error {trackingError:F6} on {name}. Cumulative={cumulativeReward:F6}, tracked={totalTracked:F6}");
        }
    }

    private string GetEpisodeReasonName(EpisodeEndReason reason)
    {
        switch (reason)
        {
            case EpisodeEndReason.Success:
                return "success";
            case EpisodeEndReason.Timeout:
                return "timeout";
            case EpisodeEndReason.Boundary:
                return "boundary";
            case EpisodeEndReason.Collision:
                return "collision";
            case EpisodeEndReason.Stuck:
                return "stuck";
            default:
                return "other";
        }
    }

    private void ResetTrainingEpisodeMetrics()
    {
        episodeDecisionCount = 0;
        episodeBallSeenCount = 0;
        episodeReverseCount = 0;
        episodeGasAbsSum = 0f;
        episodeSteeringAbsSum = 0f;
        episodeMinBallDistance = 1f;
    }

    private void RecordTrainingDecisionMetrics(float gas, float steering, bool ballSeen, float ballDistance)
    {
        if (!isTraining) return;

        episodeDecisionCount++;
        episodeGasAbsSum += Mathf.Abs(gas);
        episodeSteeringAbsSum += Mathf.Abs(steering);
        if (gas < -0.1f) episodeReverseCount++;

        if (ballSeen)
        {
            episodeBallSeenCount++;
            episodeMinBallDistance = Mathf.Min(episodeMinBallDistance, Mathf.Clamp01(ballDistance));
        }
    }

    private void LogInferenceActionIfNeeded(float gas, float steering)
    {
        if (!logInferenceActions || isTraining) return;

        inferenceActionLogCounter++;
        if (inferenceActionLogCounter % 25 != 0) return;

        float distance = GetBallDistanceMeters();
        float speed = rb != null ? rb.linearVelocity.magnitude : 0f;
        Debug.Log(string.Format(
            CultureInfo.InvariantCulture,
            "[Inference] distance={0:F2} gas={1:F2} steering={2:F2} speed={3:F2}",
            distance,
            gas,
            steering,
            speed));
    }

    private float GetBallDistanceMeters()
    {
        if (spawnedBall == null)
        {
            spawnedBall = FindLocalBall();
        }

        if (spawnedBall != null)
        {
            return Vector3.Distance(transform.position, spawnedBall.transform.position);
        }

        if (realVision != null && realVision.seesBall)
        {
            return realVision.normalizedDistance * realVision.maxViewDistance;
        }

        if (simulatedYolo != null && simulatedYolo.seesBall)
        {
            return simulatedYolo.normalizedDistance * simulatedYolo.maxViewDistance;
        }

        if (virtualCamera != null && virtualCamera.seesBall)
        {
            return virtualCamera.normalizedDistance * virtualCamera.maxViewDistance;
        }

        return -1f;
    }

    private void FinishTrainingEpisode(string reason, bool success, float finalDistance)
    {
        if (!isTraining) return;

        float decisionCount = Mathf.Max(1, episodeDecisionCount);
        float ballSeenRatio = episodeBallSeenCount / decisionCount;
        float reverseRatio = episodeReverseCount / decisionCount;
        float meanAbsGas = episodeGasAbsSum / decisionCount;
        float meanAbsSteering = episodeSteeringAbsSum / decisionCount;
        float clampedFinalDistance = Mathf.Clamp01(finalDistance);

        Academy.Instance.StatsRecorder.Add("Custom/EpisodeSuccess", success ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("Custom/EpisodeLength", StepCount);
        Academy.Instance.StatsRecorder.Add("Custom/BallSeenRatio", ballSeenRatio);
        Academy.Instance.StatsRecorder.Add("Custom/ReverseRatio", reverseRatio);
        Academy.Instance.StatsRecorder.Add("Custom/MeanAbsGas", meanAbsGas);
        Academy.Instance.StatsRecorder.Add("Custom/MeanAbsSteering", meanAbsSteering);
        Academy.Instance.StatsRecorder.Add("Custom/FinalBallDistance", clampedFinalDistance);
        Academy.Instance.StatsRecorder.Add("Custom/MinBallDistance", episodeMinBallDistance);
        RecordParallelEpisodeSuccessRate(success);

        if (!enableTrainingCsvLog) return;

        EnsureTrainingCsvWriter();
        if (trainingCsvWriter == null) return;

        trainingCsvRows++;
        Vector3 pos = transform.position;
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0},{1},{2},{3},{4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12},{13},{14:F4},{15:F4}",
            trainingCsvRows,
            System.DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            GetEntityId(),
            reason,
            success ? 1 : 0,
            GetCumulativeReward(),
            clampedFinalDistance,
            episodeMinBallDistance,
            ballSeenRatio,
            reverseRatio,
            meanAbsGas,
            meanAbsSteering,
            StepCount,
            currentActionLatency,
            pos.x,
            pos.z);
        trainingCsvWriter.WriteLine(line);
        trainingCsvWriter.Flush();
    }

    private static void EnsureTrainingCsvWriter()
    {
        if (trainingCsvWriter != null) return;

        try
        {
            string root = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            Directory.CreateDirectory(root);
            int pid = System.Diagnostics.Process.GetCurrentProcess().Id;
            trainingCsvPath = Path.Combine(root, $"training_episodes_{pid}.csv");
            trainingCsvWriter = new StreamWriter(trainingCsvPath, false, Encoding.UTF8);
            trainingCsvWriter.WriteLine("row,utc,agent_id,reason,success,cumulative_reward,final_ball_distance,min_ball_distance,ball_seen_ratio,reverse_ratio,mean_abs_gas,mean_abs_steering,steps,action_latency,robot_x,robot_z");
            trainingCsvWriter.Flush();
            Debug.Log($"[RobotBrain] Training episode CSV: {trainingCsvPath}");
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"[RobotBrain] Не удалось создать training CSV log: {ex.Message}");
            trainingCsvWriter = null;
        }
    }

    private void ResetBall(bool useInferenceTestPosition = false)
    {
        if (spawnedBall == null)
        {
            spawnedBall = FindLocalBall();
        }

        if (spawnedBall != null)
        {
            // Возвращаем в тренировочную зону, если он был в клешне
            if (spawnedBall.transform.parent != transform.parent)
            {
                spawnedBall.transform.SetParent(transform.parent);
            }

            Rigidbody ballRb = spawnedBall.GetComponent<Rigidbody>();
            if (ballRb != null)
            {
                ballRb.isKinematic = false;
                ballRb.linearVelocity = Vector3.zero;
                ballRb.angularVelocity = Vector3.zero;
            }

            // --- Domain Randomization мяча (паттерн Ball3DAgent) ---
            float ballScale = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_scale", 0.04f);
            ballScale *= UnityEngine.Random.Range(0.8f, 1.2f); // ±20% разброс
            float ballRadius = ballScale * 0.5f;

            Rigidbody bRb = spawnedBall.GetComponent<Rigidbody>();
            if (bRb != null)
            {
                bRb.mass = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_mass", 0.1f);
                bRb.mass *= UnityEngine.Random.Range(0.5f, 2.0f); // ±100% разброс
            }
            spawnedBall.transform.localScale = Vector3.one * ballScale;

            Vector3 randomPos = transform.position;
            bool validPos = false;
            int attempts = 0;

            if (useInferenceTestPosition)
            {
                randomPos = transform.position
                    + transform.right * inferenceBallLocalOffset.x
                    + Vector3.up * inferenceBallLocalOffset.y
                    + transform.forward * inferenceBallLocalOffset.z;
                validPos = IsValidBallSpawnPosition(ref randomPos, ballRadius);
            }

            while (!validPos && attempts < 30)
            {
                float minDist = Mathf.Max(0f, ballSpawnMinRadius);
                float maxDist = Mathf.Max(minDist, ballSpawnMaxRadius);

                // === 360° SPAWN: мяч появляется в ЛЮБОМ направлении вокруг робота ===
                float spawnAngle = Random.Range(0f, 360f);
                float spawnDist = Random.Range(minDist, maxDist);
                Vector3 direction = Quaternion.Euler(0f, spawnAngle, 0f) * Vector3.forward;
                randomPos = transform.position + direction * spawnDist;
                validPos = IsValidBallSpawnPosition(ref randomPos, ballRadius);
                attempts++;
            }

            if (!validPos)
            {
                // Fallback stays near the robot; choose the first clear cardinal point.
                Vector3[] fallbackDirections =
                {
                    transform.forward,
                    transform.right,
                    -transform.right,
                    -transform.forward
                };

                foreach (Vector3 direction in fallbackDirections)
                {
                    randomPos = transform.position + direction.normalized * Mathf.Max(0.7f, ballSpawnMinRadius);
                    if (IsValidBallSpawnPosition(ref randomPos, ballRadius))
                    {
                        validPos = true;
                        break;
                    }
                }
            }

            if (!validPos)
            {
                randomPos = transform.position + transform.forward * Mathf.Max(0.7f, ballSpawnMinRadius);
                PlaceBallOnFloor(ref randomPos, ballRadius);
            }

            spawnedBall.transform.position = randomPos;

            // Включаем коллайдер мяча обратно (мог быть отключен при захвате)
            Collider ballCollider = spawnedBall.GetComponent<Collider>();
            if (ballCollider != null) ballCollider.enabled = true;
        }

        if (gripper != null)
        {
            gripper.hasBall = false;
        }
    }

    private static void RefreshParallelTrainingAgentCount()
    {
        RobotBrain[] brains = FindObjectsByType<RobotBrain>(FindObjectsSortMode.None);
        int count = 0;
        foreach (RobotBrain brain in brains)
        {
            if (brain != null && brain.isTraining)
            {
                count++;
            }
        }

        parallelTrainingAgentCount = Mathf.Max(1, count);
    }

    private static void RecordParallelEpisodeSuccessRate(bool success)
    {
        if (parallelTrainingAgentCount <= 0)
        {
            RefreshParallelTrainingAgentCount();
        }

        parallelEpisodeCount++;
        if (success)
        {
            parallelEpisodeSuccessCount++;
        }

        if (parallelEpisodeCount < parallelTrainingAgentCount)
        {
            return;
        }

        float successRate = parallelEpisodeSuccessCount / (float)Mathf.Max(1, parallelEpisodeCount);
        Academy.Instance.StatsRecorder.Add("Custom/ParallelEpisodeSuccessRate", successRate);
        Academy.Instance.StatsRecorder.Add("Custom/ParallelEpisodeBatchSize", parallelEpisodeCount);
        Academy.Instance.StatsRecorder.Add("Custom/ParallelEpisodeSuccessCount", parallelEpisodeSuccessCount);
        Academy.Instance.StatsRecorder.Add("Custom/NumEnvs", parallelTrainingAgentCount);

        Debug.Log(string.Format(
            CultureInfo.InvariantCulture,
            "[ParallelTraining] success_episodes={0}/{1} success_rate={2:F2} num_envs={3}",
            parallelEpisodeSuccessCount,
            parallelEpisodeCount,
            successRate,
            parallelTrainingAgentCount
        ));

        parallelEpisodeCount = 0;
        parallelEpisodeSuccessCount = 0;
        RefreshParallelTrainingAgentCount();
    }

    private bool IsValidBallSpawnPosition(ref Vector3 position, float ballRadius)
    {
        if (!PlaceBallOnFloor(ref position, ballRadius))
        {
            return false;
        }

        float checkRadius = Mathf.Max(0.15f, ballRadius);
        Collider[] colliders = Physics.OverlapSphere(position, checkRadius);
        foreach (Collider collider in colliders)
        {
            if (collider == null) continue;
            if (collider.CompareTag("TargetBall")) continue;
            if (collider.transform == transform || collider.transform.IsChildOf(transform)) continue;

            if (collider.CompareTag("Wall") || collider.CompareTag("Obstacle"))
            {
                return false;
            }
        }

        return true;
    }

    private bool PlaceBallOnFloor(ref Vector3 position, float ballRadius)
    {
        Vector3 rayStart = new Vector3(position.x, transform.position.y + BALL_SPAWN_FLOOR_RAY_HEIGHT, position.z);
        RaycastHit[] hits = Physics.RaycastAll(rayStart, Vector3.down, BALL_SPAWN_FLOOR_RAY_DISTANCE);
        RaycastHit? floorHit = null;
        float closestDistance = float.MaxValue;

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider == null) continue;
            if (hit.collider.CompareTag("Wall") || hit.collider.CompareTag("Obstacle") || hit.collider.CompareTag("TargetBall")) continue;
            if (hit.transform == transform || hit.transform.IsChildOf(transform)) continue;

            if (hit.distance < closestDistance)
            {
                closestDistance = hit.distance;
                floorHit = hit;
            }
        }

        if (floorHit.HasValue)
        {
            position.y = floorHit.Value.point.y + ballRadius;
            return true;
        }

        position.y = transform.position.y + ballRadius;
        return false;
    }

    private void ResetBallToSceneSpawn()
    {
        if (spawnedBall == null)
        {
            spawnedBall = FindLocalBall();
        }

        if (spawnedBall != null)
        {
            if (spawnedBall.transform.parent != transform.parent)
            {
                spawnedBall.transform.SetParent(transform.parent);
            }

            Rigidbody ballRb = spawnedBall.GetComponent<Rigidbody>();
            if (ballRb != null)
            {
                ballRb.isKinematic = false;
                ballRb.linearVelocity = Vector3.zero;
                ballRb.angularVelocity = Vector3.zero;
            }

            spawnedBall.transform.position = sceneBallStartPosition;
            if (sceneBallStartScale != Vector3.zero)
            {
                spawnedBall.transform.localScale = sceneBallStartScale;
            }

            Collider ballCollider = spawnedBall.GetComponent<Collider>();
            if (ballCollider != null) ballCollider.enabled = true;
        }

        if (gripper != null)
        {
            gripper.hasBall = false;
        }
    }

    // OnCollisionEnter УБРАН — штраф теперь по ДАТЧИКАМ (Sensor Proximity)
    // На реальном роботе нет OnCollisionEnter — есть только УЗ и ИК

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;

        // По умолчанию всё стоит
        float gas = 0f;
        float steer = 0f;
        float camYaw = 0f;

        // Считываем реальное зрение (YOLO)
        bool sees = realVision != null && realVision.seesBall;
        float angle = realVision != null ? realVision.normalizedAngle : 0f;
        float dist = realVision != null ? realVision.normalizedDistance : 1f;

        if (!sees)
        {
            steer = 0.5f;
            camYaw = Mathf.Sin(Time.time * 1.5f) * 0.6f; 
        }
        else
        {
            camYaw = Mathf.Lerp(currentCameraYaw, angle, Time.deltaTime * 5f); 

            if (sensors != null && sensors.gripperIR == 1)
            {
                // Клешня сработает автоматически по ИК — просто стоим
                gas = 0f;
                steer = 0f;
                if (enableVerboseLogging) Debug.Log("[Heuristic] Мяч в клешне! Авто-захват.");
            }
            else if (dist > 0.16f)
            {
                gas = 0.45f;
                if (Mathf.Abs(angle) > 0.15f)
                {
                    steer = angle * 0.7f; 
                }
            }
            else
            {
                gas = 0.2f;
                steer = angle * 0.5f;
            }
        }

        // Записываем результат автопилота (только continuous, клешня автоматическая)
        continuousActions[0] = gas;
        continuousActions[1] = steer;
        if (continuousActions.Length > 2) continuousActions[2] = camYaw;

        // 🚨 АВАРИЙНЫЙ ПЕРЕХВАТ 🚨
        // Зажмите LEFT SHIFT чтобы управлять WASD напрямую
        if (Input.GetKey(KeyCode.LeftShift))
        {
            continuousActions[0] = Input.GetAxis("Vertical");
            continuousActions[1] = Input.GetAxis("Horizontal");
            
            if (Input.GetKey(KeyCode.Q)) continuousActions[2] = -1f;
            else if (Input.GetKey(KeyCode.E)) continuousActions[2] = 1f;
        }
    }
}
