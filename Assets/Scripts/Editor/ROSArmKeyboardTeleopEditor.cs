#if UNITY_EDITOR
using System;
using System.Globalization;
using System.Text;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ROSArmKeyboardTeleop))]
public sealed class ROSArmKeyboardTeleopEditor : Editor
{
    static readonly string[] CalibrationProperties =
    {
        "m_Script",
        "armRotationServo",
        "shoulderServo",
        "elbowServo",
        "clawServo",
        "cameraTiltServo",
        "cameraPanServo",
        "clawOpenAngle",
        "clawCubeAngle",
        "clawBallAngle",
        "logServoSnapshotOnStart",
        "logServoChanges",
        "servoLogIntervalSeconds",
        "servoLogThresholdDegrees"
    };

    SerializedProperty armRotation;
    SerializedProperty shoulder;
    SerializedProperty elbow;
    SerializedProperty claw;
    SerializedProperty cameraTilt;
    SerializedProperty cameraPan;
    SerializedProperty clawOpen;
    SerializedProperty clawCube;
    SerializedProperty clawBall;
    SerializedProperty logSnapshotOnStart;
    SerializedProperty logServoChanges;
    SerializedProperty logInterval;
    SerializedProperty logThreshold;

    bool showOtherSettings;

    ROSArmKeyboardTeleop Controller => (ROSArmKeyboardTeleop)target;

    void OnEnable()
    {
        armRotation = serializedObject.FindProperty("armRotationServo");
        shoulder = serializedObject.FindProperty("shoulderServo");
        elbow = serializedObject.FindProperty("elbowServo");
        claw = serializedObject.FindProperty("clawServo");
        cameraTilt = serializedObject.FindProperty("cameraTiltServo");
        cameraPan = serializedObject.FindProperty("cameraPanServo");
        clawOpen = serializedObject.FindProperty("clawOpenAngle");
        clawCube = serializedObject.FindProperty("clawCubeAngle");
        clawBall = serializedObject.FindProperty("clawBallAngle");
        logSnapshotOnStart = serializedObject.FindProperty("logServoSnapshotOnStart");
        logServoChanges = serializedObject.FindProperty("logServoChanges");
        logInterval = serializedObject.FindProperty("servoLogIntervalSeconds");
        logThreshold = serializedObject.FindProperty("servoLogThresholdDegrees");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.LabelField("Servo Calibration", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "Change limits and speed here while Play is stopped. During Play, use the test " +
            "buttons to move one servo smoothly. Physical limits must also match the rover master.",
            MessageType.Info
        );

        DrawToolbar();
        DrawDiagnostics();
        EditorGUILayout.Space(4f);

        DrawJoint(
            armRotation,
            "S1  Main arm",
            "2 / 3",
            () => Controller.armRotationServo
        );
        DrawJoint(
            shoulder,
            "S2  Elbow",
            "F / G",
            () => Controller.shoulderServo
        );
        DrawJoint(
            elbow,
            "S3  Claw rotation",
            "Q / E",
            () => Controller.elbowServo
        );
        DrawJoint(
            claw,
            "S4  Claw grip",
            "C / V / Z",
            () => Controller.clawServo
        );
        DrawClawPresets();
        DrawJoint(
            cameraTilt,
            "S7  Camera vertical",
            "I / K",
            () => Controller.cameraTiltServo
        );
        DrawJoint(
            cameraPan,
            "S8  Camera horizontal",
            "J / L",
            () => Controller.cameraPanServo
        );
        if (GUILayout.Button("Apply wider camera range (S7/S8 only)"))
        {
            serializedObject.ApplyModifiedProperties();
            Undo.RecordObject(Controller, "Apply wider GFS-X camera range");
            Controller.ApplyExpandedCameraCalibration();
            EditorUtility.SetDirty(Controller);
            serializedObject.Update();
        }

        if (HasDuplicateEnabledChannels())
        {
            EditorGUILayout.HelpBox(
                "Two enabled controls use the same physical servo channel.",
                MessageType.Error
            );
        }

        if (Application.isPlaying)
        {
            if (GUILayout.Button("Move all servos to Start angles"))
            {
                ApplyAndValidate();
                Controller.MoveAllServosToStart();
            }
        }

        EditorGUILayout.Space(6f);
        showOtherSettings = EditorGUILayout.Foldout(
            showOtherSettings,
            "ROS, Unity preview and advanced settings",
            true
        );
        if (showOtherSettings)
        {
            DrawPropertiesExcluding(serializedObject, CalibrationProperties);
        }

        if (serializedObject.ApplyModifiedProperties())
        {
            Controller.ValidateServoCalibration();
            EditorUtility.SetDirty(Controller);
        }
    }

    void DrawToolbar()
    {
        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("Restore last working values"))
            {
                serializedObject.ApplyModifiedProperties();
                Undo.RecordObject(Controller, "Apply GFS-X servo defaults");
                Controller.ApplyRecommendedServoCalibration();
                EditorUtility.SetDirty(Controller);
                serializedObject.Update();
            }

            if (GUILayout.Button("Copy rover limits"))
            {
                ApplyAndValidate();
                EditorGUIUtility.systemCopyBuffer = BuildRoverConfiguration();
                Debug.Log("GFS-X servo limits copied to the clipboard.", Controller);
            }
        }
    }

    void DrawDiagnostics()
    {
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            EditorGUILayout.LabelField("Servo logs", EditorStyles.boldLabel);
            logSnapshotOnStart.boolValue = EditorGUILayout.ToggleLeft(
                "Full snapshot when Play starts",
                logSnapshotOnStart.boolValue
            );
            logServoChanges.boolValue = EditorGUILayout.ToggleLeft(
                "Log changed commanded angles",
                logServoChanges.boolValue
            );

            using (new EditorGUI.DisabledScope(!logServoChanges.boolValue))
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.PropertyField(logInterval, new GUIContent("Interval"));
                EditorGUILayout.PropertyField(logThreshold, new GUIContent("Threshold"));
            }

            if (GUILayout.Button("Log all servo angles to Console (P in Play)"))
            {
                ApplyAndValidate();
                Controller.LogAllServoAngles();
            }
        }
    }

    void DrawJoint(
        SerializedProperty joint,
        string title,
        string keys,
        Func<ROSArmKeyboardTeleop.ServoJoint> runtimeJoint
    )
    {
        SerializedProperty enabled = joint.FindPropertyRelative("enabled");
        SerializedProperty channel = joint.FindPropertyRelative("channel");
        SerializedProperty min = joint.FindPropertyRelative("minAngle");
        SerializedProperty max = joint.FindPropertyRelative("maxAngle");
        SerializedProperty start = joint.FindPropertyRelative("startAngle");
        SerializedProperty current = joint.FindPropertyRelative("currentAngle");
        SerializedProperty speed = joint.FindPropertyRelative("degreesPerSecond");
        SerializedProperty invert = joint.FindPropertyRelative("invertInput");

        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            using (new EditorGUILayout.HorizontalScope())
            {
                enabled.boolValue = EditorGUILayout.Toggle(enabled.boolValue, GUILayout.Width(18f));
                EditorGUILayout.LabelField(title, EditorStyles.boldLabel);
                GUILayout.FlexibleSpace();
                EditorGUILayout.LabelField(keys, GUILayout.Width(72f));
                EditorGUILayout.LabelField("Channel", GUILayout.Width(52f));
                channel.intValue = EditorGUILayout.IntField(channel.intValue, GUILayout.Width(32f));
            }

            using (new EditorGUI.DisabledScope(!enabled.boolValue))
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    DrawCompactFloat("Min", min, 78f);
                    DrawCompactFloat("Max", max, 78f);
                    DrawCompactFloat("Start", start, 88f);
                    EditorGUI.BeginChangeCheck();
                    DrawCompactFloat("Current", current, 100f);
                    bool currentChanged = EditorGUI.EndChangeCheck();
                    if (currentChanged && Application.isPlaying)
                    {
                        float requestedAngle = current.floatValue;
                        serializedObject.ApplyModifiedProperties();
                        runtimeJoint().SetCurrentAngle(requestedAngle);
                        EditorUtility.SetDirty(Controller);
                        serializedObject.Update();
                    }
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    DrawCompactFloat("Speed", speed, 92f);
                    invert.boolValue = EditorGUILayout.ToggleLeft(
                        "Invert",
                        invert.boolValue,
                        GUILayout.Width(62f)
                    );
                }

                if (Application.isPlaying)
                {
                    using (new EditorGUILayout.HorizontalScope())
                    {
                        EditorGUILayout.LabelField("Smooth test", GUILayout.Width(82f));
                        if (GUILayout.Button("Min"))
                        {
                            SetTestTarget(runtimeJoint, min.floatValue);
                        }
                        if (GUILayout.Button("Start"))
                        {
                            SetTestTarget(runtimeJoint, start.floatValue);
                        }
                        if (GUILayout.Button("Max"))
                        {
                            SetTestTarget(runtimeJoint, max.floatValue);
                        }
                    }
                }
            }
        }
    }

    void DrawClawPresets()
    {
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            EditorGUILayout.LabelField("S4 grip presets", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                DrawCompactFloat("Open C/X", clawOpen, 112f);
                DrawCompactFloat("Cube V", clawCube, 100f);
                DrawCompactFloat("Ball Z", clawBall, 100f);
            }

            if (Application.isPlaying)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Open"))
                    {
                        SetTestTarget(() => Controller.clawServo, clawOpen.floatValue);
                    }
                    if (GUILayout.Button("Cube"))
                    {
                        SetTestTarget(() => Controller.clawServo, clawCube.floatValue);
                    }
                    if (GUILayout.Button("Ball"))
                    {
                        SetTestTarget(() => Controller.clawServo, clawBall.floatValue);
                    }
                }
            }
        }
    }

    static void DrawCompactFloat(string label, SerializedProperty property, float width)
    {
        using (new EditorGUILayout.HorizontalScope(GUILayout.Width(width)))
        {
            EditorGUILayout.LabelField(label, GUILayout.Width(width - 44f));
            property.floatValue = EditorGUILayout.FloatField(
                property.floatValue,
                GUILayout.Width(40f)
            );
        }
    }

    void SetTestTarget(
        Func<ROSArmKeyboardTeleop.ServoJoint> runtimeJoint,
        float angle
    )
    {
        ApplyAndValidate();
        runtimeJoint().SetTargetAngle(angle);
    }

    void ApplyAndValidate()
    {
        serializedObject.ApplyModifiedProperties();
        Controller.ValidateServoCalibration();
        EditorUtility.SetDirty(Controller);
        serializedObject.Update();
    }

    bool HasDuplicateEnabledChannels()
    {
        bool[] used = new bool[9];
        SerializedProperty[] joints =
        {
            armRotation,
            shoulder,
            elbow,
            claw,
            cameraTilt,
            cameraPan
        };

        foreach (SerializedProperty joint in joints)
        {
            if (!joint.FindPropertyRelative("enabled").boolValue)
            {
                continue;
            }

            int channel = Mathf.Clamp(
                joint.FindPropertyRelative("channel").intValue,
                1,
                8
            );
            if (used[channel])
            {
                return true;
            }
            used[channel] = true;
        }

        return false;
    }

    string BuildRoverConfiguration()
    {
        ROSArmKeyboardTeleop.ServoJoint[] joints =
        {
            Controller.armRotationServo,
            Controller.shoulderServo,
            Controller.elbowServo,
            Controller.clawServo,
            Controller.cameraTiltServo,
            Controller.cameraPanServo
        };

        StringBuilder text = new StringBuilder();
        text.AppendLine("SERVO_LIMITS = {");
        foreach (ROSArmKeyboardTeleop.ServoJoint joint in joints)
        {
            if (!joint.enabled)
            {
                continue;
            }
            text.AppendLine(string.Format(
                CultureInfo.InvariantCulture,
                "    {0}: ({1:0}, {2:0}),",
                joint.channel,
                joint.LowAngle,
                joint.HighAngle
            ));
        }
        text.AppendLine("}");
        text.AppendLine();
        text.AppendLine("SERVO_START_ANGLES = {");
        foreach (ROSArmKeyboardTeleop.ServoJoint joint in joints)
        {
            if (!joint.enabled)
            {
                continue;
            }
            text.AppendLine(string.Format(
                CultureInfo.InvariantCulture,
                "    {0}: {1:0},",
                joint.channel,
                joint.startAngle
            ));
        }
        text.AppendLine("}");
        text.AppendLine();
        text.AppendLine(string.Format(
            CultureInfo.InvariantCulture,
            "CLAW_PRESETS = {{'open': {0:0}, 'cube': {1:0}, 'ball': {2:0}}}",
            Controller.clawOpenAngle,
            Controller.clawCubeAngle,
            Controller.clawBallAngle
        ));
        return text.ToString();
    }
}
#endif
