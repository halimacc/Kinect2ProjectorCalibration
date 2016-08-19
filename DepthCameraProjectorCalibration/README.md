Calibrate Kinect2 Depth Camera and Projector
============================================
This is a Processing program used to **calibrate Kinect2 depth camera and projector**. It is modifyed from the calibration example of [KinectProjectorToolkit](https://github.com/genekogan/KinectProjectorToolkit).

System Requirement
------------------
- Windows 8+
- [Microsoft Kinect SDK 2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561) installed

Installation
------------
1. Install [Processing 3](https://processing.org/).
2. Open Processing, selecting "Add Library..." from the "Import Library..." submenu within the Sketch menu, search and install `ControlP5`, `OpenCV for processing`, and `Kinect v2 for Processing`.
3. Clone [KinectProjectorToolkit](https://github.com/genekogan/KinectProjectorToolkit) into your Processing library folder (usually Document->Processing->library).
4. Clone this repository to your computer, and open it with Processing.

Calibration
-----------
Basiclly follow the [tutorial video](https://vimeo.com/84658886) provided by original author, but with some **notes** below.

- Before calibration, modify following paramters at the top of `DepthCameraProjectorCalibration.pde` to your specific environment:

	1. `ProjectorScreenId`: the idendity number of projector screen (mostly 1 or 2)
	2. `pWidth` and `pHeight`: size of projector screen
	3. `DepthCameraParams`: intrinsics of your Kinect2 depth camera, you can calibrate it yourself or you can get it by using the [CameraIntrinsicsTool](./../CameraIntrinsicsTool).

- Get 3 or more boards for each board position you choose. this can reduce the impact of Kinect depth image shake.

Usage of Calibrated data
------------------------

if you want to fully understand the usage of calibration data, refer to [this blog](http://blog.3dsense.org/programming/kinect-projector-calibration-human-mapping-2/). 

- Depth Space → Camera Space
	
	**Note**: before coordinate transformation, you have to mirror Kinect depth image.

        Point depth2camera(int w, int h, float depth) {
			Point point;
	    	point.z = depth;
			point.x = (x - CameraParams.cx) * point.z / CameraParams.fx;
			point.y = (CameraParams.cy - y) * point.z / CameraParams.fy;
			return point;
    	}

- Camera Space → Projector Space
	
	**note**: `t0` to `t10` is the 11 float numbers in calibration file, and project space here is in same coordinate with viewport space.
		
		Point camera2projector(Point cp) {
			Point point;
			point.z = 0;
			float denom = t8 * cp.x + t9 * cp.y + t10 * cp.z + 1;
			point.x = 2 * (t0 * cp.x + t1 * cp.y + t2 * cp.z + t3) / denom - 1;
			point.y = 1 - 2 * (t4 * cp.x + t5 * cp.y + t6 * cp.z + t7) / denom;
			return point;
		}

 