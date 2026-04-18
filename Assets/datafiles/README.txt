Two-camera RealSense point cloud setup

This folder holds the first-phase Unity scene and placeholder data files for two Intel RealSense D455 cameras.

Files:
- TwoCameraPointCloudSetup.unity: fresh scene with two RsDevice branches and two separate point clouds
- camera_serials.template.json: place to note the two camera serial numbers and layout defaults

Workflow:
1. Open TwoCameraPointCloudSetup.unity
2. Select the TwoCameraPointCloudSetup root object
3. Paste the two RealSense serial numbers into cameraASerial and cameraBSerial
4. Connect both cameras over USB 3
5. Press Play and verify two RGB point clouds appear side by side
