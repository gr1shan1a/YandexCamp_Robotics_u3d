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
///   1 Idle   2 Reach   3 Carry
///   Q / E — roll локтя
///   Z / C — сомкнуть / раскрыть губки
///   Space — захват мяча (Close),  X — отпустить (Open)
///   R — полный сброс
/// </summary>
public class ArmKeyboardTester : MonoBehaviour
{
    public GripperController gripper;
    public float rollInputSpeed = 1.5f;
    public bool logKeyPresses = true;

    float m_Roll;
    bool warnedMissingGripper;

    void Awake()
    {
        if (gripper == null) gripper = GetComponentInChildren<GripperController>();
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

        bool k1, k2, k3, kQ, kE, kZ, kC, kSpace, kX, kR;

#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb == null) return;
        k1 = kb.digit1Key.wasPressedThisFrame;
        k2 = kb.digit2Key.wasPressedThisFrame;
        k3 = kb.digit3Key.wasPressedThisFrame;
        kQ = kb.qKey.isPressed;
        kE = kb.eKey.isPressed;
        kZ = kb.zKey.wasPressedThisFrame;
        kC = kb.cKey.wasPressedThisFrame;
        kSpace = kb.spaceKey.wasPressedThisFrame;
        kX = kb.xKey.wasPressedThisFrame;
        kR = kb.rKey.wasPressedThisFrame;
#else
        k1 = Input.GetKeyDown(KeyCode.Alpha1);
        k2 = Input.GetKeyDown(KeyCode.Alpha2);
        k3 = Input.GetKeyDown(KeyCode.Alpha3);
        kQ = Input.GetKey(KeyCode.Q);
        kE = Input.GetKey(KeyCode.E);
        kZ = Input.GetKeyDown(KeyCode.Z);
        kC = Input.GetKeyDown(KeyCode.C);
        kSpace = Input.GetKeyDown(KeyCode.Space);
        kX = Input.GetKeyDown(KeyCode.X);
        kR = Input.GetKeyDown(KeyCode.R);
#endif

        if (k1)
        {
            gripper.SetPose(GripperController.ArmPose.Idle);
            LogCommand("1 -> Idle");
        }

        if (k2)
        {
            gripper.SetPose(GripperController.ArmPose.Reach);
            LogCommand("2 -> Reach");
        }

        if (k3)
        {
            gripper.SetPose(GripperController.ArmPose.Carry);
            LogCommand("3 -> Carry");
        }

        float dir = (kE ? 1f : 0f) - (kQ ? 1f : 0f);
        if (dir != 0f)
        {
            m_Roll = Mathf.Clamp(m_Roll + dir * rollInputSpeed * Time.deltaTime, -1f, 1f);
            gripper.SetWristRoll(m_Roll);
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
            m_Roll = 0f;
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
            "1 Idle  2 Reach  3 Carry\n" +
            "Q/E — roll кисти\n" +
            "Z/C — сомкнуть / раскрыть губки\n" +
            "Space — захват,  X — отпустить,  R — сброс\n" +
            (gripper != null
                ? $"Поза: {gripper.CurrentPose}  Доехала: {gripper.PoseReached}\n" +
                  $"Губки сомкнуты: {gripper.JawsClosed}  Мяч: {gripper.HasBall}"
                : "GripperController не найден!"));
    }
}
