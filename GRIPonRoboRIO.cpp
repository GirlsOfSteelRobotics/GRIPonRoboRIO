/*
 * GRIPonRoboRIO
 *
 * Usage: GRIPonRoboRIO [-v] [-f FPS] [-c frameCount] [-e exposure%] [-s image.jpg] [-p port]
 */

#include <iostream>
#include <time.h>
#include <unistd.h>

#include "opencv2/core.hpp"
#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "GripPipeline.h"
using namespace std;

#define MAXCONTOURS 5

int main(int argc, char** argv)
{
    grip::GripPipeline pipeline;
    vector<vector<cv::Point> > *contours;
    shared_ptr<NetworkTable> table;
    vector<double> center_xs;
    vector<double> center_ys;
    vector<double> widths;
    vector<double> heights;

    // Parse command line options
    int verbose = 0;
    int framesPerSec = 10;
    int frameCount = -1;
    int exposurePercent = 16;
    char *saveFilename = NULL;
    int streamPort = -1;
    int opt;
    while ((opt = getopt(argc, argv, "vhf:c:e:s:p:")) != -1) {
	switch (opt) {
	case 'v':
	    verbose++;
	    break;
	case 'f':
	    framesPerSec = atoi(optarg);
	    break;
	case 'c':
	    frameCount = atoi(optarg);
	    break;
	case 'e':
	    exposurePercent = atoi(optarg);
	    break;
	case 's':
	    saveFilename = optarg;
	    break;
	case 'p':
	    streamPort = atoi(optarg);
	    break;
	case 'h':
	default:
	    cerr << "Usage: " << argv[0] << 
		" [-v] [-f FPS] [-c frameCount] [-e exposure%] [-s image.jpg] [-p port]" << endl;
	    cerr << "  -v            Verbose mode, dumps out contour values" << endl;
	    cerr << "  -f FPS        Frames per second, defaulting to 10 (7, 10, 20, 30 are valid)" << endl;
	    cerr << "  -c count      Limit this run to count frames, defaulting to -1 for unlimited" << endl;
	    cerr << "  -e exposure%  0 - 100, but only changes exposures at 0, 8, 16, 24, ... 96, and 100" << endl;
	    cerr << "  -s image.jpg  Save the second captured image to the filename" << endl;
	    cerr << "  -p port       Publish an MJPEG stream for GRIP troubleshooting on the port number" << endl;
	    return 1;
	}
    }

    // Get ready to use USB camera 0
    cs::UsbCamera camera{"usbcam", 0};
    // Set the resolution and exposure
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, framesPerSec);
    camera.SetExposureManual(exposurePercent);
    // Get a CvSink for capturing frames from the camera
    cs::CvSink cvSink{"opencv_USB Camera 0"};
    cvSink.SetSource(camera);
    // OpenCV matrix to hold a frame of video
    cv::Mat frame;

    // Optionally start an MJPEG stream, useful as an input to GRIP for pipeline troubleshooting
    if (streamPort > 0) {
	cerr << "Starting an MJPEG stream on port " << streamPort << endl;
	cs::MjpegServer server{"serve_USB Camera 0", streamPort};
	server.SetSource(camera);
    }

    // Establish connection to Network Tables server
    NetworkTable::SetClientMode();
    NetworkTable::SetIPAddress("localhost");
    NetworkTable::Initialize();
    table = NetworkTable::GetTable("GRIP/myContoursReport");

    // Construct a vision processing pipeline
    pipeline = grip::GripPipeline();

    time_t start = time(NULL);

    // Run the pipeline forever to a certain frame count
    int count;
    for (count = 0; frameCount == -1 || count < frameCount; count++) {
	size_t imgsize;
	void *imgdata;
	center_xs.clear();
	center_ys.clear();
	widths.clear();
	heights.clear();
	// This blocks until a frame is available, per the camera FPS setting
	// We usually don't have a frame the very first time, while the camera is being set up
	if (cvSink.GrabFrame(frame) == 0 || frame.empty()) {
	    count--;
	    continue;
	}
	// If requested with -s, save the second image (the first one doesn't have exposure set correctly)
	if (count == 1 && saveFilename) {
	  cerr << "Saving second image to: " << saveFilename << endl;
	  cv::imwrite(saveFilename, frame);
	  saveFilename = NULL;
	}
	pipeline.Process(frame);
	//cv::imwrite("imagethresh.jpg", *(pipeline.GetHsvThresholdOutput()));
	contours = pipeline.GetFilterContoursOutput();
	int i = 0;
	for (vector<cv::Point> contour: *contours) {
	    cv::Rect bb = cv::boundingRect(contour);
	    double area = contourArea(contour);
	    if (verbose)
		cout << i << ": " << bb << ", area = " << area << "\n";
	    center_xs.push_back(bb.x + (bb.width / 2.0));
	    center_ys.push_back(bb.y + (bb.height / 2.0));
	    widths.push_back(bb.width);
	    heights.push_back(bb.height);
	    // If we found too many contours, just skip the rest
	    if (i++ >= MAXCONTOURS)
		break;
	}
	if (i == 0) {
	    if (verbose)
		cout << "No contours found\n";
	}
	table->PutNumberArray("centerX", center_xs);
	table->PutNumberArray("centerY", center_ys);
	table->PutNumberArray("width", widths);
	table->PutNumberArray("height", heights);
    }

    // If we got here, a -c frame count was given, so we can give an approx frame rate
    time_t stop = time(NULL);
    cerr << "Iterations: " << frameCount << endl;
    cerr << "Elapsed seconds: " << (stop - start) << endl;
    cerr << "FPS: " << ((double)frameCount / (stop - start)) << endl;

    return 0;
}
