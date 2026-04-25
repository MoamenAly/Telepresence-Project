using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(RsTwoCameraPointCloudSetup))]
public class RsTwoCameraPointCloudSetupEditor : Editor
{
    private const float PositionStepFine = 0.005f;
    private const float PositionStepCoarse = 0.02f;
    private const float RotationStepFine = 0.5f;
    private const float RotationStepCoarse = 2f;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var setup = (RsTwoCameraPointCloudSetup)target;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Calibration Tools", EditorStyles.boldLabel);

        if (GUILayout.Button("Apply Full Setup"))
            RunAndRecord(setup, setup.ApplyFullSetup);

        if (GUILayout.Button("Calculate Symmetric Start"))
            RunAndRecord(setup, () =>
            {
                setup.useSymmetricStartingLayout = true;
                setup.CalculateSymmetricAlignment();
                setup.ApplyPointCloudLayout();
            });

        if (GUILayout.Button("Auto Align From Live Frames"))
            RunAndRecord(setup, setup.BeginAutomaticAlignment);

        if (GUILayout.Button("Switch To Manual Alignment"))
            RunAndRecord(setup, () =>
            {
                setup.useSymmetricStartingLayout = false;
                setup.ApplyPointCloudLayout();
            });

        EditorGUILayout.HelpBox(setup.GetAutoCalibrationStatus(), MessageType.Info);

        EditorGUILayout.Space();
        DrawPositionControls(setup);
        EditorGUILayout.Space();
        DrawRotationControls(setup);
    }

    private static void DrawPositionControls(RsTwoCameraPointCloudSetup setup)
    {
        EditorGUILayout.LabelField("Position Nudge (meters)", EditorStyles.boldLabel);
        DrawAxisRow(
            setup,
            "X",
            new Vector3(-PositionStepCoarse, 0f, 0f),
            new Vector3(-PositionStepFine, 0f, 0f),
            new Vector3(PositionStepFine, 0f, 0f),
            new Vector3(PositionStepCoarse, 0f, 0f));
        DrawAxisRow(
            setup,
            "Y",
            new Vector3(0f, -PositionStepCoarse, 0f),
            new Vector3(0f, -PositionStepFine, 0f),
            new Vector3(0f, PositionStepFine, 0f),
            new Vector3(0f, PositionStepCoarse, 0f));
        DrawAxisRow(
            setup,
            "Z",
            new Vector3(0f, 0f, -PositionStepCoarse),
            new Vector3(0f, 0f, -PositionStepFine),
            new Vector3(0f, 0f, PositionStepFine),
            new Vector3(0f, 0f, PositionStepCoarse));
    }

    private static void DrawRotationControls(RsTwoCameraPointCloudSetup setup)
    {
        EditorGUILayout.LabelField("Rotation Nudge (degrees)", EditorStyles.boldLabel);
        DrawAxisRow(
            setup,
            "Rot X",
            new Vector3(-RotationStepCoarse, 0f, 0f),
            new Vector3(-RotationStepFine, 0f, 0f),
            new Vector3(RotationStepFine, 0f, 0f),
            new Vector3(RotationStepCoarse, 0f, 0f),
            true);
        DrawAxisRow(
            setup,
            "Rot Y",
            new Vector3(0f, -RotationStepCoarse, 0f),
            new Vector3(0f, -RotationStepFine, 0f),
            new Vector3(0f, RotationStepFine, 0f),
            new Vector3(0f, RotationStepCoarse, 0f),
            true);
        DrawAxisRow(
            setup,
            "Rot Z",
            new Vector3(0f, 0f, -RotationStepCoarse),
            new Vector3(0f, 0f, -RotationStepFine),
            new Vector3(0f, 0f, RotationStepFine),
            new Vector3(0f, 0f, RotationStepCoarse),
            true);
    }

    private static void DrawAxisRow(
        RsTwoCameraPointCloudSetup setup,
        string label,
        Vector3 coarseNegative,
        Vector3 fineNegative,
        Vector3 finePositive,
        Vector3 coarsePositive,
        bool rotation = false)
    {
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.PrefixLabel(label);

        if (GUILayout.Button(rotation ? "-2" : "-2cm"))
            Nudge(setup, coarseNegative, rotation);

        if (GUILayout.Button(rotation ? "-0.5" : "-5mm"))
            Nudge(setup, fineNegative, rotation);

        if (GUILayout.Button(rotation ? "+0.5" : "+5mm"))
            Nudge(setup, finePositive, rotation);

        if (GUILayout.Button(rotation ? "+2" : "+2cm"))
            Nudge(setup, coarsePositive, rotation);

        EditorGUILayout.EndHorizontal();
    }

    private static void Nudge(RsTwoCameraPointCloudSetup setup, Vector3 delta, bool rotation)
    {
        RunAndRecord(setup, () =>
        {
            if (rotation)
                setup.NudgeManualAlignment(Vector3.zero, delta);
            else
                setup.NudgeManualAlignment(delta, Vector3.zero);
        });
    }

    private static void RunAndRecord(RsTwoCameraPointCloudSetup setup, System.Action action)
    {
        Undo.RecordObject(setup, "Adjust Two Camera Alignment");

        if (setup.pointCloudRootB != null)
            Undo.RecordObject(setup.pointCloudRootB, "Adjust Point Cloud Root B");

        action();

        EditorUtility.SetDirty(setup);
        if (setup.pointCloudRootB != null)
            EditorUtility.SetDirty(setup.pointCloudRootB);
    }
}
