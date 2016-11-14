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
#include "gnuplot_i.hpp"

using namespace std;
using namespace cv;


/*
const int8_t FOVX = 104;   // 60deg = 1.04 rad
const int8_t FOVY = 79;    // 45deg = 0.785 rad
 */

#define FOVX 104
#define FOVY 79



void plot_line_gnu(double A, double B, uint16_t size, Gnuplot *g, bool hold_on, string title);
void plot_gnu(int32_t *array, uint16_t size, Gnuplot *g, bool hold_on, string title);

int main()
{

  // initialize structures for plottin
  Gnuplot g("lines");
  //parameters for edgeflow
  struct edgeflow_parameters_t edgeflow_parameters;
  struct edgeflow_results_t edgeflow_results;
  int32_t edge_histogram_x[IMAGE_WIDTH];
  //initialize for edgeflow
  edgeflow_init(&edgeflow_parameters, &edgeflow_results, FOVX, FOVY, 128, 96, 0);

  char name[10];
  int i = 1;

  //structures for images
  Rect ROI_right(0, 0, 128, 94); //Note that in the database, image left and right is reverted!
  Rect ROI_left(128, 0, 128, 94);
  Mat image_left, image_left_gray;
  Mat image_right, image_right_gray;
  Mat image;
  uint8_t image_buffer[128 * 96 * 2];
  uint8_t image_buffer_left[128 * 96 ];
  uint8_t image_buffer_right[128 * 96 ];

  memset(image_buffer, 0, 128 * 96 * 2 * sizeof(uint8_t));

  // open cvs file
  ofstream output;
  // open video capture
  VideoCapture cap;
  cap.open("stereoboard_database/Take16/%1d.bmp");
  output.open("stereoboard_database/Take16/result.csv");
  edgeflow_parameters.stereo_shift = -5; // calibration data of that file

  /*  cap.open("stereoboard_database/Track3/%1d.bmp");
  output.open("stereoboard_database/Track3/result.csv");
  edgeflow_parameters.stereo_shift = 2;*/

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
    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image_left_gray );
    //waitKey(0);
    // Put image values in array, just like in the stereoboard
    int x, y, idx, idx2;
    for (y = 0; y < image.rows; y++) {
      for (x = 0; x < image.cols; x++) {
        idx = 2 * (128 * y + x);
        idx2 = (128 * y + x);

        //TODO: this should be the right order?
        image_buffer_left[idx2] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer_right[idx2] = (uint8_t)image_right_gray.at<uchar>(y, x);
        image_buffer[idx] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer[idx + 1] = (uint8_t)image_right_gray.at<uchar>(y, x);
      }
    }

    //dummyvalues
    int16_t *stereocam_data;
    uint8_t *edgeflowArray;

    //calculate edgeflow
    edgeflow_total(edgeflowArray, stereocam_data, 0, image_buffer,
                   &edgeflow_parameters, &edgeflow_results);

    // Plot results
    plot_gnu(edgeflow_results.displacement.x, 128, &g, true, "displacement.x");
    double A = (double)edgeflow_results.edge_flow.div_x / 100;
    double B = (double)(edgeflow_results.edge_flow.flow_x + edgeflow_results.edge_flow.div_x * (128 / 2)) / 100;
    plot_line_gnu(A, B, 128, &g, false, "edgeflow");
    getchar();

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
void plot_gnu(int32_t *array, uint16_t size, Gnuplot *g, bool hold_on, string title)
{
  static int plot_counter = 0;

  std::vector<double> X;
  std::vector<double> Y;

  int x;
  for (x = 0; x < size; x++) {
    X.push_back((double)x);
    Y.push_back((double)array[x]);
  }

  g->set_style("lines").plot_xy(X, Y, title);

  if (hold_on == false) {
    g->reset_plot();
  }

  if (plot_counter > 40) { //TODO: get rid of those irritating warnings...
    g->remove_tmpfiles();
    plot_counter = 0;
  } else {
    plot_counter++;
  }

}

void plot_line_gnu(double A, double B, uint16_t size, Gnuplot *g, bool hold_on, string title)
{
  static int plot_counter = 0;

  std::vector<double> X;
  std::vector<double> Y;
  int x;
  for (x = 0; x < size; x++) {
    X.push_back((double)x);
    Y.push_back((double)(A * x + B));
  }

  g->set_style("lines").plot_xy(X, Y, title);

  if (hold_on == false) {
    g->reset_plot();
  }

  if (plot_counter > 40) { //TODO: get rid of those irritating warnings...
    g->remove_tmpfiles();
    plot_counter = 0;
  } else {
    plot_counter++;
  }

}
