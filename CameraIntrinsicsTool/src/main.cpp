#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <fstream>
using namespace libfreenect2;
using namespace std;

int main() {
	ofstream output("camera_intrinsics.yml");
	
	Freenect2 freenect;
	int deviceCount = freenect.enumerateDevices();
	// output intrinsics of the first device
	if (deviceCount > 1)
		deviceCount = 1;

	for (int idx = 0; idx < deviceCount; ++idx) {
		string serial = freenect.getDeviceSerialNumber(idx);
		PacketPipeline *pipeline = new OpenGLPacketPipeline();
		Freenect2Device *device = freenect.openDevice(serial, pipeline);
		if (device == 0) {
			continue;
		}
		device->start();
		Freenect2Device::IrCameraParams irParams = device->getIrCameraParams();
		output << "depth:\n";
		output << "  fx: " << irParams.fx << endl;
		output << "  fy: " << irParams.fy << endl;
		output << "  cx: " << irParams.cx << endl;
		output << "  cy: " << irParams.cy << endl;

		Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams();
		output << "color:\n";
		output << "  fx: " << colorParams.fx << endl;
		output << "  fy: " << colorParams.fy << endl;
		output << "  cx: " << colorParams.cx << endl;
		output << "  cy: " << colorParams.cy << endl;

		device->stop();
		device->close();
	}

	output.flush();
	output.close();
}