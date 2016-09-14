#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
//Include header file of stereoboard code
#include "../../stereoboard/edgeflow.h"
#include "gnuplot_i.hpp"

using namespace std;
using namespace cv;

/*
const int8_t FOVX = 104;   // 60deg = 1.04 rad
const int8_t FOVY = 79;    // 45deg = 0.785 rad
*/

#define FOVX 1.001819   // 57.4deg = 1.001819 rad
#define FOVY 0.776672    // 44.5deg = 0.776672 rad


void plot_edge_histogram(int32_t *edge_histogram);

int main(int argc, const char **argv)
{
  Gnuplot g1("lines");
  Gnuplot g2("lines");
  Gnuplot g3("lines");
  Gnuplot g4("lines");

	//parameters for edgeflow
	struct edgeflow_parameters_t edgeflow_parameters;
	struct edgeflow_results_t edgeflow_results;

	//initialize for edgeflow
	edgeflow_init(&edgeflow_parameters, &edgeflow_results, FOVX, FOVY, 128, 94, 0);

	char name[10];
	int i = 1;

	//structures for images
	Rect ROI_right(0, 0, 128, 94); //Note that in the database, image left and right is reversed!
	Rect ROI_left(128, 0, 128, 94);
	Mat image_left_gray;
	Mat image_right_gray;
	Mat image;
	uint8_t image_buffer[128 * 94 * 2];
	uint8_t image_buffer_left[128 * 94 ];
	uint8_t image_buffer_right[128 * 94 ];

	// open cvs file
	ofstream output;
	// open video capture
	/*VideoCapture cap("stereoboard_database/Take16/%d.bmp");
	output.open("stereoboard_database/Take16/result.csv");
	edgeflow_parameters.stereo_shift = 2; // calibration data of that file*/

	VideoCapture cap("stereoboard_database/Track3/%d.bmp");
  output.open("stereoboard_database/Track3/result.csv");
  edgeflow_parameters.stereo_shift = -4;

	if (!cap.isOpened()) return -1;

	//start loop while images are read
	int counter = 0;
	for(;;) {
		counter++;
		cap >> image;

		if (image.empty()) {
			break;
		}

		namedWindow( "image", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "image", image );                   // Show our image inside it.
    waitKey(1);

		// Crop out the seperate left and right images
		if (image.channels() == 3) {
			cvtColor(image(ROI_left), image_left_gray, COLOR_BGR2GRAY);
			cvtColor(image(ROI_right), image_right_gray, COLOR_BGR2GRAY);
		} else {
			image_left_gray = image(ROI_left);
			image_right_gray = image(ROI_right);
		}

		// Put image values in array, just like in the stereoboard
		int x, y, idx,idx2;
		for (y = 0; y < image_left_gray.rows; y++) {
			for (x = 0; x < image_left_gray.cols; x++)
			{
				idx = 2 * image_left_gray.cols * y + x;
				idx2 = image_left_gray.cols * y + x;

				image_buffer_left[idx2] = (uint8_t)image_left_gray.at<uchar>(y, x);
				image_buffer_right[idx2] = (uint8_t)image_right_gray.at<uchar>(y, x);
				image_buffer[idx] = (uint8_t)image_left_gray.at<uchar>(y, x);
				image_buffer[idx + image_left_gray.cols] = (uint8_t)image_right_gray.at<uchar>(y, x);
			}
		}

		//dummyvalues
		int16_t *stereocam_data;
		uint8_t *edgeflowArray;

		//calculate edgeflow
		edgeflow_total(stereocam_data, 0, image_buffer, &edgeflow_parameters, &edgeflow_results);
		std::vector<double> X ;
		std::vector<double> Y1, Y2, Y3, Y4, Y5, Y6, prev_hist, curr_hist, dist_hist;

		double grad, y_int;
    grad = edgeflow_results.edge_flow.scaled_flow_z;
    y_int = edgeflow_results.edge_flow.scaled_flow_x;

    double global_grad = (double)edgeflow_results.edge_flow.div_x * edgeflow_results.avg_dist / 100.;
    double global_y_int = (double)edgeflow_results.edge_flow.flow_x * edgeflow_results.avg_dist / 100.;

    printf("frame: %d\n", counter);
    printf("%d %f %d %f\n", edgeflow_results.vel_x_pixelwise, y_int, edgeflow_results.vel_z_pixelwise, grad);
    printf("%d %f %d %f\n\n", edgeflow_results.edge_flow.flow_x, global_y_int, edgeflow_results.edge_flow.div_x, global_grad);

    printf("%d %d %d %d\n", edgeflow_results.edge_flow.flow_x, edgeflow_results.vel_x_global, edgeflow_results.edge_flow.div_x, edgeflow_results.vel_z_global);

    printf("avg_disp %d avg_dist %d\n", edgeflow_results.avg_disp, edgeflow_results.avg_dist);

    double error = 0, global_error = 0;
		for(x=0; x < 128; x++)
		{
			X.push_back((double)x);

			Y1.push_back((double)edgeflow_results.velocity_per_column[x]*100.);
			Y2.push_back((double)(y_int + grad * (x-64) ));
			Y5.push_back(1000*(double)!edgeflow_results.faulty_distance[x]);
			Y6.push_back((double)!edgeflow_results.faulty_distance[x]);

			if(!edgeflow_results.faulty_distance[x])
			  error += fabs(Y1[x] - Y2[x]);

			Y3.push_back((double)edgeflow_results.displacement.x[x] * edgeflow_results.avg_dist * 100.);
			Y4.push_back((double)(global_y_int + global_grad * (x-64) ));
			if(x > 23 && x < 128 - 23)
			  global_error += fabs(Y3[x] - Y4[x]);

			prev_hist.push_back(edgeflow_results.edge_hist[edgeflow_results.previous_frame_x].x[x]);
			curr_hist.push_back(edgeflow_results.edge_hist[(edgeflow_results.current_frame_nr - 1 + MAX_HORIZON) % MAX_HORIZON].x[x]);

			dist_hist.push_back(edgeflow_results.stereo_distance_per_column[x]/100.);
		}

		printf("pixelwise plot error: %f, global plot error: %f\n", error, global_error);

		g1.set_grid();
		g2.set_grid();
		g3.set_grid();
		g4.set_grid();

		g1.plot_xy(X, Y1, "velocity_per_column");
		g1.plot_xy(X, Y2, "Flow fit");
		g1.plot_xy(X, Y5, "Good Distance");

		g2.plot_xy(X, Y3, "pixel displacement per column");
		g2.plot_xy(X, Y4, "Flow fit");

		g3.plot_xy(X, prev_hist, "prev_hist");
		g3.plot_xy(X, curr_hist, "curr_hist");

		g4.plot_xy(X, dist_hist, "distance histogram");
		g4.plot_xy(X, Y6, "Good Distance");
		getchar();

		g1.reset_plot();
		g2.reset_plot();
		g3.reset_plot();
		g4.reset_plot();

		//Save data on output.cvs
		//TODO: also enter groundtruth data
		output << edgeflow_results.vel_x_pixelwise << "," << edgeflow_results.vel_z_pixelwise <<
				", " << edgeflow_results.vel_x_global << "," << edgeflow_results.vel_y_global <<
				"," << edgeflow_results.vel_z_global << "," << edgeflow_results.velocity_stereo_mean <<
				"," << edgeflow_results.vel_x_stereo_avoid_pixelwise << "," << edgeflow_results.vel_z_stereo_avoid_pixelwise
				<< ", " << edgeflow_results.avg_dist << endl;
	}

	output.close();

	return 0;
}

// Edgeflow function to plot some variables in a window
//TODO: find a nice plotting library for this
void plot_edge_histogram(int32_t *edge_histogram)
{
	int size = 128;

	cv::Mat edge_histogram_plot = cv::Mat::zeros(128, size, CV_8UC3);
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



