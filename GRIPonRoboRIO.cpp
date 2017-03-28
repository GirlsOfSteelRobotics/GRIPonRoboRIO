/*
 * GRIPonRoboRIO
 *
 * Usage: GRIPonRoboRIO [-v] [-f FPS] [-c frameCount] [-e exposure%]
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
    int opt;
    while ((opt = getopt(argc, argv, "vf:c:e:")) != -1) {
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
	default:
	    cerr << "Usage: " << argv[0] << " [-v] [-f FPS] [-c frameCount] [-e exposure%]" << endl;
	    cerr << "  -v            Verbose mode, dumps out contour values" << endl;
	    cerr << "  -f FPS        Frames per second, defaulting to 10" << endl;
	    cerr << "  -c count      Limit this run to count frames, defaulting to -1 for unlimited" << endl;
	    cerr << "  -e exposure%  0 - 100, but only changes exposures at 0, 8, 16, 24, ... 96, 100" << endl;
	    exit(EXIT_FAILURE);
	}
    }

    // Get ready to use USB camera 0
    cs::UsbCamera camera{"usbcam", 0};
    // Set the resolution and exposure
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, framesPerSec);
    // TODO: Setting exposure appears to have no effect!
    camera.SetExposureManual(exposurePercent);
    // Get a CvSink for capturing frames from the camera
    cs::CvSink cvSink{"opencv_USB Camera 0"};
    cvSink.SetSource(camera);
    // OpenCV matrix to hold a frame of video
    cv::Mat frame;

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
	//cv::imwrite("image.jpg", frame);
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

    time_t stop = time(NULL);
    cerr << "Iterations: " << frameCount << endl;
    cerr << "Elapsed seconds: " << (stop - start) << endl;
    cerr << "FPS: " << ((double)frameCount / (stop - start)) << endl;

    return 0;
}
