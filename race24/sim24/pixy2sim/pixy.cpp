//source ./venv/bin/activate
// g++ pixy.cpp -fPIC -shared fw.cpp misc.cpp -o pixy2sim.so -I/usr/include/python3.10 && python3 setup.py install

//on windows: https://kezunlin.me/post/8b9c051d/
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <cstdio>
#include <chrono>
#include <functional>
#include <string>

#include "fw.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
void process_frame(py::array_t<uint8_t> &msg, uint32_t ts, py::array_t<uint8_t> &out) {
	micro_timestamp = ts;
	py::buffer_info info = msg.request();
	uint8_t*ptr=static_cast<uint8_t*>(info.ptr);
	//printf("timestamp %d size %ld val %hhd %hhd %hhd\n", ts, info.size, *ptr, *(ptr+1), *(ptr+2));
	//printf("size %ld\n", info.size);
	//printf("val %hhd\n", *ptr);	
	loop2(ptr);
	py::buffer_info out_info = out.request();
	const int out_sz = 318*208;
	if(out_sz == out_info.size) {
		uint8_t*optr=static_cast<uint8_t*>(out_info.ptr);
		//memcpy(optr, SRAM1_LOC+CAM_PREBUF_LEN, out_sz);
		for(int r=0;r<CAM_RES3_HEIGHT;r++) for(int c=0;c<CAM_RES3_WIDTH;c++) 
			optr[(2*r+1-(c&1))*(CAM_RES3_WIDTH>>1)+(c>>1)] = CAM_M0R3_PTR[r*CAM_RES3_WIDTH+c];
	}
}

void set_vector_callback(line_detect_event_t line_callback) {
	vector_detected = line_callback;
}

void set_line_callback(line_detect_event_t line_callback) {
	line_detected = line_callback;
}
void set_intersection_callback(intersection_detect_event_t intersection_callback) {
	intersection_detected = intersection_callback;
}
/*int main(int argc, char ** argv) {
	return 0;
}

int main(int argc, char ** argv) {
	printf("hello world pixy_sim package!!!\n");
	line_open();
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Pixy2SimNode>());
	printf("bye bye!\n");
	rclcpp::shutdown();
	line_close();
	return 0;
}
*/

void test_uint8_vect(py::buffer b) {
	 py::buffer_info info = b.request();
		if (info.format != py::format_descriptor<unsigned char>::format())
				throw std::runtime_error("Expected a bytearray!");
		uint8_t *ptr = static_cast<uint8_t *>(info.ptr);
		size_t length = info.shape[0];
		for(int i=0;i<length;i++) printf("%d ", ptr[i]);
		printf("\n");
		ptr[0]=10;
}

PYBIND11_MODULE(pixy2sim, m) {
	m.def("line_open", &line_open, R"phbdoc(
		Used to initialize Pixy2 line module (memory resources and default firmware parameters).

		Needs to be called  at least once, before using any other module function.
	)phbdoc");
	m.def("set_vector_callback", &set_vector_callback, R"phbdoc(
		Used to attach a callback function for the active vector.
		It is invoked at most once during calls to process_frame function. 

		Callback is auto detached when calling line_close function in order to avoid segmentation faults due to undestroyed objects. 
	)phbdoc");
	m.def("set_line_callback", &set_line_callback, R"phbdoc(
		Used to attach a callback function for aquiring details about each detected line.
		It is invoked once for each detected line (if any) during calls to process_frame function. 

		Callback is auto detached when calling line_close function in order to avoid segmentation faults due to undestroyed objects.
	)phbdoc");
	m.def("set_intersection_callback", &set_intersection_callback, R"phbdoc(
		Used to attach a callback function for aquiring details about each detected intersection.
		It is invoked once for each detected intersection (if any) during calls to process_frame function. 

		Callback is auto detached when calling line_close function in order to avoid segmentation faults due to undestroyed objects.
	)phbdoc");
	m.def("set_pixy2_param", &line_shadowCallback, py::arg("key"), py::arg("value"), R"phbdoc(
		Used to set Pixy2 camera line detection parameters.
		Key must be case sensitive, identical to one of the keys from the .prm config file exported with the PixyMon.
		Values are represented as unsigned 32bit, but internally they may be trimmed down to 8 or 16 bit.
		Returns 0 if parameter key was found, 1 otherwise.
		
		params_with_demo_values = {
			# line detection params (standard)
			"Edge distance": 4, #UINT16 g_dist
			"Edge threshold": 150, #UINT16 g_thresh => g_hThresh
			"Minimum line width": 2, #UINT16 g_minLineWidth
			"Maximum line width": 60, #UINT16 g_maxLineWidth
			"Line extraction distance": 13, #UINT16 g_extractionDist
			"Maximum merge distance": 0, #UINT16 g_maxMergeDist
			"Minimum line length": 21, #UINT16 g_minLineLength =>g_minLineLength2 (squared)
			"Maximum line compare": 7280, #UINT32 g_maxLineCompare
			"White line": 0, #UINT8 g_whiteLine
			"Line filtering": 1, #UINT8 g_lineFiltering
			"Intersection filtering": 1, #UINT8 g_primaryIntersection.setTiming(leading, trailing);
			"Manual vector select": 0, #UINT8 g_manualVectorSelect

			# params not available through standard Pixy2 API, but manually exposed for research
            "g_maxSegTanAngle": math.tan(math.pi / 4) * 1000, # g_maxSegTanAngle (angle between two segments under which are considered single segment)
            "g_maxEquivTanAngle": math.tan(math.pi / 10) * 1000,  #
            "g_maxTrackingTanAngle": math.tan(math.pi / 10) * 1000,
            "g_pointsPerSeg": 12,

			# extra commands (not params, but part of the Pixy2 API)
			"line_setVector": 0, #UINT8 (activates manual vector select and sets the line index to become main vector)
			"line_reversePrimary": 0 #UINT8 (parameter is ignored, sending the setting will cause Pixy API call to reverse vector orientation)

			# unsupported line detection params (standard on Pixy2, but not supported in this version of the simulator)
			#"Barcode filtering": 1, #UINT8 g_barcodeFiltering (barcode not supported)
			#"Default turn angle": 0, #UINT16
			#"Delayed turn": 0, #UINT8 g_delayedTurn
		}
		)phbdoc");
	m.def("process_frame", &process_frame, py::arg("img"), py::arg("micros"), py::arg("out")=NULL, R"pbdoc(
		Used to process one camera frame and trigger lines, intersetions and vector callbacks, if registered.
		
		First parameter is pointer to an numpy uint8_t array with shape [416 lines, 636 cols, 3 channels] containing image to be processed in BGR color channel order (actually not important since only the center green channel is sampled by the actual firmware...).
		Only lines with numbers in {4*l, 4*l+1} for l in {0..103} are actually used, and only green channel (1) of every second pixel is sampled (to emulate the camera Bayer sensor layout)and reduce resolution to 318 pixels per line).
		
		Second parameter is the simulation time in microseconds for the moment of the shot (needed to emulate time base filtering in Pixy2 for vectors and intersections).
		
		Third parameter is optional and is only used to get the actual sampled monochrome image if the provided array has the length precisly 208*318.
		It can be useful for diagnostic purposes because the resolution reduction might have adverse effects on fine details in original image.

		If features are detected (lines, vectors, intersections) and appropriate callbacks were registered, then the specific callback will be invoked for each individual item that is detected before the function execution is completed.
	)pbdoc");
	m.def("line_close", &line_close, R"phbdoc(
		Used to deinitialize Pixy2 line module before exit.

		Frees Pixy2 simulator memory allocation and cleans all attached callbacks to avoid unexpected segmentation faults on exit.
	)phbdoc");
	m.def("test_uint8_vect", &test_uint8_vect, R"phbdoc(
		Test simple bytearray passing and updating.

		Usage example:
		vec = bytearray([0] * (16))
		pixy2sim.test_uint8_vect(vec)
		print(list(vec))
	)phbdoc");
	m.attr("__version__")="dev";
}
