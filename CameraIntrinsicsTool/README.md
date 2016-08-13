Kinect2 Camera Intrinsics Tool
==============================
This is an executable file using [libfreenect2](https://github.com/OpenKinect/libfreenect2) to get the camera intrinsics of Kinect2 under Windows 8+.

Instruction
-----------
1. Install latest x64 [UsbDk](https://github.com/daynix/UsbDk/releases) driver.
2. Clone this repository to your computer, and copy `freenect2.dll`, `glfw3.dll`, `libusb-1.0.dll`, `turbojpeg.dll` from src\libfreenect2\lib\ to same directory with `CameraIntrinsicsTool.exe`.
3. Connect your Kinect2 device and run `CameraIntrinsicsTool.exe`, then you can find `camera_intrinsics.yml` under the same directory.

