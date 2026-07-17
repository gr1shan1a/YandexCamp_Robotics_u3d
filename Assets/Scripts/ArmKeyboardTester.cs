using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Тестер руки с клавиатуры — БЕЗ ML-Agents.
/// Вешается на робота рядом с GripperController.
/// Обычный Play (без mlagents-learn) → проверяй позы/roll/губки/захват.
///
/// Клавиши:
///   1 Idle
///   Hold 2 — lower arm, hold 3 — raise arm; release to stop
///   Hold F — bend elbow, hold G — straighten elbow; release to stop
///   Q — wrist 0 degrees, E — wrist 90 degrees around rotatePoint
///   Z / C — сомкнуть / раскрыть губки
///   Space — захват мяча (Close),  X — отпустить (Open)
///   R — полный сброс
/// </summary>
public class ArmKeyboardTester : MonoBehaviour
{
    public GripperController gripper;
    public bool logKeyPresses = true;

    bool warnedMissingGripper;
    bool armWasMoving;
    bool elbowWasMoving;

    void Awake()
    {
        GripperController primary = GripperController.FindController(transform.root);
        if (primary != null)
        {
            gripper = primary;
        }
    }

    void Update()
    {
        if (gripper == null)
        {
            if (!warnedMissingGripper)
            {
                Debug.LogWarning("ArmKeyboardTester: GripperController не найден. Перетащи компонент GripperController в поле Gripper.");
                warnedMissingGripper = true;
            }

            return;
        }

        bool k1, k2, k3, kF, kG, kQ, kE, kZ, kC, kSpace, kX, kR;

#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb == null) return;
        k1 = kb.digit1Key.wasPressedThisFrame;
        k2 = kb.digit2Key.isPressed;
        k3 = kb.digit3Key.isPressed;
        kF = kb.fKey.isPressed;
        kG = kb.gKey.isPressed;
        kQ = kb.qKey.wasPressedThisFrame;
        kE = kb.eKey.wasPressedThisFrame;
        kZ = kb.zKey.wasPressedThisFrame;
        kC = kb.cKey.wasPressedThisFrame;
        kSpace = kb.spaceKey.wasPressedThisFrame;
        kX = kb.xKey.wasPressedThisFrame;
        kR = kb.rKey.wasPressedThisFrame;
#else
        k1 = Input.GetKeyDown(KeyCode.Alpha1);
        k2 = Input.GetKey(KeyCode.Alpha2);
        k3 = Input.GetKey(KeyCode.Alpha3);
        kF = Input.GetKey(KeyCode.F);
        kG = Input.GetKey(KeyCode.G);
        kQ = Input.GetKeyDown(KeyCode.Q);
        kE = Input.GetKeyDown(KeyCode.E);
        kZ = Input.GetKeyDown(KeyCode.Z);
        kC = Input.GetKeyDown(KeyCode.C);
        kSpace = Input.GetKeyDown(KeyCode.Space);
        kX = Input.GetKeyDown(KeyCode.X);
        kR = Input.GetKeyDown(KeyCode.R);
#endif

        if (k1)
        {
            gripper.SetPose(GripperController.ArmPose.Idle);
            armWasMoving = false;
            LogCommand("1 -> Idle");
        }

        float armInput = (k3 ? 1f : 0f) - (k2 ? 1f : 0f);
        if (armInput != 0f)
        {
            gripper.JogArm(armInput, Time.deltaTime);
            if (!armWasMoving)
            {
                LogCommand(armInput < 0f ? "2 held -> lower arm" : "3 held -> raise arm");
            }

            armWasMoving = true;
        }
        else if (armWasMoving && !k1)
        {
            gripper.StopArm();
            armWasMoving = false;
            LogCommand("2/3 released -> hold current position");
        }

        float elbowInput = (kG ? 1f : 0f) - (kF ? 1f : 0f);
        if (elbowInput != 0f)
        {
            gripper.JogElbow(elbowInput, Time.deltaTime);
            if (!elbowWasMoving)
            {
                LogCommand(elbowInput < 0f ? "F held -> bend elbow" : "G held -> straighten elbow");
            }

            elbowWasMoving = true;
        }
        else if (elbowWasMoving)
        {
            gripper.StopElbow();
            elbowWasMoving = false;
            LogCommand("F/G released -> hold elbow position");
        }

        if (kQ)
        {
            gripper.SetWristAngleDegrees(0f);
            LogCommand("Q -> wrist 0 degrees");
        }

        if (kE)
        {
            gripper.SetWristAngleDegrees(90f);
            LogCommand("E -> wrist 90 degrees");
        }

        if (kZ)
        {
            gripper.CloseJaws();
            LogCommand("Z -> CloseJaws");
        }

        if (kC)
        {
            gripper.OpenJaws();
            LogCommand("C -> OpenJaws");
        }

        if (kSpace)
        {
            gripper.Close();
            LogCommand("Space -> Close/TryGrab");
        }

        if (kX)
        {
            gripper.Open();
            LogCommand("X -> Open/Release");
        }

        if (kR)
        {
            gripper.ResetState();
            armWasMoving = false;
            elbowWasMoving = false;
            LogCommand("R -> Reset");
        }
    }

    void LogCommand(string message)
    {
        if (!logKeyPresses)
        {
            return;
        }

        Debug.Log($"ArmKeyboardTester: {message}");
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 460, 160),
            "Тест руки (без ML-Agents):\n" +
            "1 — Idle\n" +
            "Удерживай 2/3 — опустить/поднять; отпусти — стоп\n" +
            "Удерживай F/G — согнуть/разогнуть локоть; отпусти — стоп\n" +
            "Q — кисть 0°, E — кисть 90° вокруг rotatePoint\n" +
            "Z/C — сомкнуть / раскрыть губки\n" +
            "Space — захват,  X — отпустить,  R — сброс\n" +
            (gripper != null
                ? $"Поза: {gripper.CurrentPose}  Доехала: {gripper.PoseReached}\n" +
                  $"Локоть: {gripper.CurrentElbowAngle:F1}°  Готов: {gripper.ElbowReady}\n" +
                  $"Губки сомкнуты: {gripper.JawsClosed}  Мяч: {gripper.HasBall}"
                : "GripperController не найден!"));
    }
}
