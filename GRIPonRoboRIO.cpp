#include <iostream>
#include "opencv2/core.hpp"
#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "GripPipeline.h"
using namespace std;

#define MAXCONTOURS 5

int main(int argc, char** argv)
{
    bool verbose = 0;
    grip::GripPipeline pipeline;
    vector<vector<cv::Point> > *contours;
    shared_ptr<NetworkTable> table;
    vector<double> center_xs;
    vector<double> center_ys;
    vector<double> widths;
    vector<double> heights;

    // Establish connection to Network Tables server
    NetworkTable::SetClientMode();
    NetworkTable::SetIPAddress("localhost");
    NetworkTable::Initialize();
    table = NetworkTable::GetTable("GRIP/myContoursReport");

    // Get ready to use USB camera 0
    cs::UsbCamera camera{"usbcam", 0};
    // Set the resolution and exposure
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
    camera.SetExposureManual(19);
    // Get a CvSink for capturing frames from the camera
    cs::CvSink cvSink{"opencv_USB Camera 0"};
    cvSink.SetSource(camera);
    // OpenCV matrix to hold a frame of video
    cv::Mat frame;

    pipeline = grip::GripPipeline();

    // Run the pipeline forever
    int count;
    for(count = 0; /*count < 2*/; count++)
    {
          size_t imgsize;
          void *imgdata;
	  center_xs.clear();
	  center_ys.clear();
	  widths.clear();
	  heights.clear();
	  if (cvSink.GrabFrame(frame) == 0) {
	      continue;
	  }
	  //cv::imwrite("image.jpg", frame);
          if (frame.empty())
	      break; // end of video stream
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

    return 0;
}
