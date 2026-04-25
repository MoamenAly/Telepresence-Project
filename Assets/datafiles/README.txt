Two-camera RealSense point cloud setup

This folder holds the Unity scene and placeholder data files for two Intel RealSense D455 cameras.

Files:
- TwoCameraPointCloudSetup.unity: scene with two RsDevice branches, clipped point clouds, and Camera B alignment into Camera A
- camera_serials.template.json: place to note serial numbers, clipping defaults, and initial alignment values

Workflow:
1. Open TwoCameraPointCloudSetup.unity
2. Select the TwoCameraPointCloudSetup root object
3. Paste the two RealSense serial numbers into cameraASerial and cameraBSerial
4. Connect both cameras over USB 3
5. Set baselineMeters to the measured distance between the two camera centers. Use the exact measured value, not the range.
6. If the cameras are angled inward, set toeInYawDegrees to one camera's inward angle. For a 20-30 degree physical range, start at the measured angle or 25 degrees, then tune in 1 degree steps.
7. Press Play and verify Camera B's point cloud appears in Camera A's space
8. For manual fine tuning, disable useSymmetricStartingLayout and edit cameraBPositionInA / cameraBRotationInAEuler. If Camera B appears too far left/right, tune X. If it appears too near/far, tune Z. If the overlap twists around the vertical axis, tune rotation Y.
