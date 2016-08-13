Calibrate Kinect2 Depth Camera and Projector
===============================
This is a Processing program used to **calibrate Kinect2 depth camera and projector**. It is modifyed from the calibration example of [KinectProjectorToolkit](https://github.com/genekogan/KinectProjectorToolkit).

System Requirement
------------------
- Windows 8+
- [Microsoft Kinect SDK 2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561) installed

Installation
------------
1. Install [Processing 3](https://processing.org/).
2. Open Processing, selecting "Add Library..." from the "Import Library..." submenu within the Sketch menu, search and install `Kinect Projector Toolkit`, `ControlP5`, `OpenCV for processing`, and `Kinect v2 for Processing`.
3. Clone this repository to your computer, and open this directory with Processing.

Calibration
-----------
Basiclly follow the [tutorial video](https://vimeo.com/84658886) provided by original author, but with some **notes** below.

- Before calibration, modify following paramters at the top of `DepthCameraProjectorCalibration.pde` to your specific environment:

	1. `ProjectorScreenId`: the idendity number of projector screen (mostly 1 or 2)
	2. `pWidth` and `pHeight`: size of projector screen
	3. `DepthCameraParams`: intrinsics of your kinect2 depth camera, you can calibrate it yourself or you can get it by using the [CameraIntrinsicsTool](./../CameraIntrinsicsTool).

- Get 3 or more boards for each board position you choose. this can reduce the impact of Kinect depth image shake.

