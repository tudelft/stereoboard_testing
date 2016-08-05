#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
//Include header file of stereoboard code
#include "../stereoboard/edgeflow.h"


using namespace std;
using namespace cv;

/*
const int8_t FOVX = 104;   // 60deg = 1.04 rad
const int8_t FOVY = 79;    // 45deg = 0.785 rad
*/

#define FOVX 1.04
#define FOVY 0.785

void plot_edge_histogram(int32_t *edge_histogram);

int main(int, char **)
{

  //parameters for edgeflow
  struct edgeflow_parameters_t edgeflow_parameters;
  struct edgeflow_results_t edgeflow_results;
  int32_t edge_histogram_x[IMAGE_WIDTH];
  //initialize for edgeflow
  edgeflow_init(&edgeflow_parameters, &edgeflow_results, FOVX, FOVY, IMAGE_WIDTH, IMAGE_HEIGHT, 0);

  char name[10];
  int i = 1;

  //structures for images
  Rect ROI_left(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT-2);
  Rect ROI_right(IMAGE_WIDTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT-2);
  Mat image_left, image_left_gray;
  Mat image_right, image_right_gray;
  Mat image;
  uint8_t image_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
  memset(image_buffer, 0, IMAGE_WIDTH * IMAGE_HEIGHT * 2 * sizeof(uint8_t));

  // open rate file
  ifstream angles("stereoboard_database/Track3/angles.log");
  std::string line_str; 
  
  int16_t stereocam_data[18];
  stereocam_data[2] = 0;  // derotation
  stereocam_data[3] = 1;  // adaptive_horizon
  stereocam_data[4] = 0;  // window_size
  stereocam_data[5] = 0;  // disparity_range
  stereocam_data[6] = 0;  // snap_shot
  stereocam_data[7] = 0;  // kalman_on
  stereocam_data[8] = 0;  // autopilot_mode
  
  uint8_t stereo_len;
  if (angles.is_open()){
    stereo_len = 18;
  } else {
    stereo_len = 0;
  }
  
  // open cvs file
  ofstream output;
  // open video capture
  VideoCapture cap;
  /*    cap.open("stereoboard_database/Take16/%1d.bmp");
       output.open("stereoboard_database/Take16/result.csv");
          edgeflow_parameters.stereo_shift = -5;*/

  cap.open("stereoboard_database/Track3/%1d.bmp");
  output.open("stereoboard_database/Track3/result.csv");
  edgeflow_parameters.stereo_shift = 2;

  //start loop while images are read
  int counter = 0;
  while (cap.isOpened()) {

    counter++;
    cap.read(image);

    if (image.empty()) {
      break;
    }
    // Crop out the seperate left and right images
    if (image.channels() == 3) {
      image_left = image(ROI_left);
      image_right = image(ROI_right);
      cvtColor(image_left, image_left_gray, COLOR_BGR2GRAY);
      cvtColor(image_right, image_right_gray, COLOR_BGR2GRAY);
    } else {
      image_left_gray = image(ROI_left);
      image_right_gray = image(ROI_right);
    }

    // Put image values in array, just like in the stereoboard
    int x, y, idx;
    for (y = 0; y < image.rows; y++) {
      for (x = 0; x < image.cols; x++)
      {
        idx = 2 * (IMAGE_WIDTH * y + x);
        //TODO: this should be the right order?
        image_buffer[idx + 1] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer[idx] = (uint8_t)image_right_gray.at<uchar>(y, x);
      }
    }

    cv::namedWindow("video", CV_WINDOW_NORMAL);
    cv::imshow("video", image_left_gray);
    cv::waitKey(1);

    if (std::getline(angles, line_str))
    {
      char myString[256];
      strcpy(myString, line_str.c_str());
      char *p = strtok(myString, " ");

      stereocam_data[0] = (int16_t)(std::stof(p)*256);  // phi
      p = strtok(NULL, " ");
      stereocam_data[1] = (int16_t)(std::stof(p)*256);  // theta
    }
    //dummyvalues
    //uint8_t *edgeflowArray;

    //calculate edgeflow
    edgeflow_total(stereocam_data, stereo_len, image_buffer,
                   &edgeflow_parameters, &edgeflow_results);

    /*    plot_edge_histogram(edgeflow_results.displacement.stereo);
          cv::waitKey(0);*/

//Save data on output.cvs
//TODO: also enter groundtruth data
    output << (int)edgeflow_results.vel_x_pixelwise << "," << (int)edgeflow_results.vel_z_pixelwise <<
           ", " << (int)edgeflow_results.vel_x_global << "," << (int)edgeflow_results.vel_y_global <<
           "," << (int)edgeflow_results.vel_z_global << "," << (int)edgeflow_results.velocity_stereo_mean <<
           "," << (int)edgeflow_results.vel_x_stereo_avoid_pixelwise << "," << (int)edgeflow_results.vel_z_stereo_avoid_pixelwise
           << ", " << (int)edgeflow_results.avg_dist << endl;
  }

  output.close();

  return 0;
}

// Edgeflow function to plot some variables in a window
//TODO: find a nice plotting library for this
void plot_edge_histogram(int32_t *edge_histogram)
{
  int size = IMAGE_WIDTH;

  cv::Mat edge_histogram_plot = cv::Mat::zeros(IMAGE_WIDTH, size, CV_8UC3);
  cv::Point line1, line2;
  cv::Point line1_prev, line2_prev;
  cv::Point line1_disp, line2_disp;

  line1.x = 0;
  line1.y = 0;

  for (int x = 1; x < size; x++) {

    line2.x = x;
    line2.y = (int)edge_histogram[x] / 10 + 60;


    cv::line(edge_histogram_plot, line1, line2, Scalar(0, 255, 0));



    line1.x = line2.x;
    line1.y = line2.y;

    line1_prev.x = line2_prev.x;
    line1_prev.y = line2_prev.y;


  }


  flip(edge_histogram_plot, edge_histogram_plot, 0);
  cv::namedWindow("edge_histogram", CV_WINDOW_NORMAL);

  cv::imshow("edge_histogram", edge_histogram_plot);


}



