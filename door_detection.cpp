#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
//Include header file of stereoboard code
#include "stereoboard/gate_detection.h"
#include "stereoboard/main_parameters.h"
#include "gnuplot_i.hpp"

#include "image.h"

using namespace std;
using namespace cv;

void generate_mux_image(Mat* image_l, Mat* image_r, uint8_t* image_buffer_left, uint8_t* image_buffer_right, uint8_t* image_buffer);
void gate_gray_detector(struct image_t* input, Mat* grad);
void gate_gradient_detector(struct image_t* input, Mat* grad);


uint16_t bin_cnt[3] = {0};
uint16_t bin_cnt_snake[3] = {0};
struct roi_t roi;

struct gate_t gate;

bool make_movie = true;
const string video_name = "./door_video.avi";
clock_t t_start, t_end;

uint32_t positive = 0, negative = 0;

#define COLOR    0
#define GRAY     1
#define GRADIENT 2

#define IMAGE_TYPE GRAY

void initialise_gate_settings(void)
{
  roi.tl.x = 2;// (IMAGE_WIDTH - cal_width) / 2;
  roi.tl.y = 2;//(IMAGE_HEIGHT - cal_height) / 2;
  roi.br.x = IMAGE_WIDTH-2;//(IMAGE_WIDTH + cal_width) / 2;
  // if we only want to look at possible floor points that are more than 7 meters away
  // we can exclude some of the points in the bottom of the frame
  // namely 16 deg of the FOV resulting in only looking at the top 62 pixels
  roi.br.y = IMAGE_HEIGHT - 2;//(IMAGE_HEIGHT+cal_height)/2;

#if IMAGE_TYPE == GRADIENT
  gate_set_intensity(50, 255); // nus meeting room
#elif IMAGE_TYPE == GRAY
  gate_set_intensity(5, 108); // nus meeting room
#else
  gate_set_color(30, 190, 90, 140, 160, 255); // red
#endif
}

int main(int argc, const char **argv)
{
  initialise_gate_settings();

	//structures for images
	Mat image_l;
	Mat image_l_big;
	Mat image_r;
	Mat vid_frame(384, 1024, CV_8UC1, Scalar(0,0,0));
	uint8_t image_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * 2] = {0};
	uint8_t image_buffer_left[IMAGE_WIDTH * IMAGE_HEIGHT] = {0};
	uint8_t image_buffer_right[IMAGE_WIDTH * IMAGE_HEIGHT] = {0};

	Mat output(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, Scalar(0,0,0));
	Mat output_big(384, 512, CV_8UC1, Scalar(0,0,0));

	// open video capture
	VideoCapture cap_l("/home/kirk/mavlab/SmartUAV/MATLAB/DelFlyInvestigator/rooms_dataset/singapore_rooms/left%d.png");
	VideoCapture cap_r("/home/kirk/mavlab/SmartUAV/MATLAB/DelFlyInvestigator/rooms_dataset/singapore_rooms/right%d.png");

  Mat prev_image;
  char prev_image_file[255];

	if (!cap_l.isOpened() || !cap_r.isOpened()) return -1;

	int counter = 0;

	cap_l >> image_l;
  cap_r >> image_r;
  counter++;

  VideoWriter outputVideo;                                        // Open the output
  if(make_movie){
    int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
    outputVideo.open(video_name, codec, 6., vid_frame.size(), false);

    // check if we succeeded
    if (!outputVideo.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }
  }

  float total_time = 0, total_gates = 0, total_q = 0;

  /* initialize random seed: */
  srand (time(NULL));

	// start loop while images are read
	for(;;) {
    counter++;
    cap_l >> image_l;
    cap_r >> image_r;

		if (image_l.empty() || image_r.empty()) {
			break;
		}

		// Crop out the separate left and right images
    if (image_l.channels() == 3) {
      cvtColor(image_l, image_l, COLOR_BGR2GRAY);
      cvtColor(image_r, image_r, COLOR_BGR2GRAY);
    }

    namedWindow( "image", WINDOW_AUTOSIZE );  // Create a window for display.
    imshow( "image", image_r );               // Show our image inside it.

    resize(image_l, image_l, Size(), 0.5, 0.5);
    resize(image_r, image_r, Size(), 0.5, 0.5);

    generate_mux_image(&image_l, &image_r, image_buffer_left, image_buffer_right, image_buffer);

    struct image_t input;
    input.buf = image_buffer_left;
    input.w = IMAGE_WIDTH;
    input.h = IMAGE_HEIGHT;
    input.buf_size = IMAGE_WIDTH*IMAGE_HEIGHT;
    input.type = IMAGE_GRAYSCALE;

#if IMAGE_TYPE == GRADIENT
    gate_gradient_detector(&input, &output);
#elif IMAGE_TYPE == GRAY
    gate_gray_detector(&input, & output);
#else

#endif

    resize(output, output_big, Size(), 4, 4);
    namedWindow( "image", WINDOW_AUTOSIZE );      // Create a window for display.
    imshow( "after", output_big);                   // Show our image inside it.

    resize(image_l, image_l_big, Size(), 4, 4);
    for (int y = 0; y < output_big.rows; y++) {
      for (int x = 0; x < output_big.cols; x++) {
        vid_frame.at<uchar>(y, x) = image_l_big.at<uchar>(y,x);
        vid_frame.at<uchar>(y, x + output_big.cols) = output_big.at<uchar>(y,x);
      }
    }

    waitKey(1);

    if (make_movie){
      // encode the frame into the videofile stream
      outputVideo.write(vid_frame);
    }

    if (gate.q > 15)
    {
      printf("frame: %d %f\n", counter, ((double) (t_end - t_start)) * 1000 / CLOCKS_PER_SEC);
      total_time += ((double) (t_end - t_start)) * 1000 / CLOCKS_PER_SEC;
      total_gates++;
      total_q += gate.q;
      getchar();
    }

    /*if (gate.q > 15){
      char key = std::cin.get();
      printf("%d\n", key);
      if (key == 49){
        positive++;
      } else if(key == 48){
        negative++;
      }
	  }*/
	}
	printf("%f total gates, %d positivly identified doors. %d negative\n", total_gates, positive, negative);
	if (total_gates)
	  printf("Avergage computation time: %f ms, total q: %f\n", total_time / total_gates, total_q);
	return 0;
}

void generate_mux_image(Mat* image_l, Mat* image_r, uint8_t* image_buffer_left, uint8_t* image_buffer_right, uint8_t* image_buffer)
{
  // Put image values in array, just like in the stereoboard
  int x, y, idx,idx2;

  for (y = 0; y < image_l->rows; y++) {
    for (x = 1; x < image_l->cols; x++)  //stored images have coloums shifted by one, first coloum is actually last coloum
    {
      idx2 = image_l->cols * y + x - 1;

      image_buffer_left[idx2] = (uint8_t)image_l->at<uchar>(y, x);
      image_buffer_right[idx2] = (uint8_t)image_r->at<uchar>(y, x);

#if (CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux)
      idx = 2 * (image_l->cols * y + x - 1);
      image_buffer[idx] = (uint8_t)image_l->at<uchar>(y, x);
      image_buffer[idx + 1] = (uint8_t)image_r->at<uchar>(y, x);
#elif (CAMERA_CPLD_STEREO == camera_cpld_stereo_linemux)
      idx = 2 * image_l->cols * y + x - 1;
      image_buffer[idx] = (uint8_t)image_l->at<uchar>(y, x);
      image_buffer[idx + image_l->cols] = (uint8_t)image_r->at<uchar>(y, x);
#endif
    }
  }

  // fill first colomn
  for (y = 0; y < image_l->rows; y++)
  {
    idx2 = image_l->cols * y + image_l->cols - 1;
    image_buffer_left[idx2] = (uint8_t)image_l->at<uchar>(y+1, 0);
    image_buffer_right[idx2] = (uint8_t)image_r->at<uchar>(y+1, 0);

#if (CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux)
    idx = 2 * (image_l->cols * y + x);
    image_buffer[idx] = (uint8_t)image_l->at<uchar>(y+1, 0);
    image_buffer[idx + 1] = (uint8_t)image_r->at<uchar>(y+1, 0);
#elif (CAMERA_CPLD_STEREO == camera_cpld_stereo_linemux)
    idx = 2 * image_l->cols * y + x;
    image_buffer[idx] = (uint8_t)image_l->at<uchar>(y+1, 0);
    image_buffer[idx + image_l->cols] = (uint8_t)image_r->at<uchar>(y+1, 0);
#endif
  }
}

void gate_gray_detector(struct image_t* input, Mat* output)
{
  memset(bin_cnt_snake, 0, sizeof(bin_cnt));  // reset the counters

  t_start = clock();
  snake_gate_detection(input, &gate, false, bin_cnt_snake, &roi, NULL);
  t_end = clock();

  struct point_t roi[2];
  roi[0].x = gate.x-gate.sz; roi[0].y = gate.y-gate.sz_left;
  roi[1].x = gate.x+gate.sz; roi[1].y = gate.y+gate.sz_left;
  //uint32_t avg = image_roi_mean(&d, roi);
  //printf("avg = %d\n", avg);
  printf("gate %d (%d,%d) %d %d %d\n", gate.q, gate.x, gate.y, gate.sz, gate.n_sides, gate.rot);

  for (int y = 0; y < input->h; y++) {
    for (int x = 0; x < input->w; x++) {
      output->at<uchar>(y, x) = ((uint8_t*)input->buf)[x + y*input->w];
    }
  }
}

void gate_gradient_detector(struct image_t* input, Mat* output)
{
  uint8_t image_grad[input->w * input->h] = {0};

  struct image_t d;
  d.buf = image_grad;
  d.w = input->w;
  d.h = input->h;
  d.buf_size = input->w*input->h;
  d.type = IMAGE_GRAYSCALE;

  image_2d_gradients(input, &d);
  //image_2d_sobel(&input, &d);

  memset(bin_cnt_snake, 0, sizeof(bin_cnt));  // reset the counters

  t_start = clock();
  snake_gate_detection(&d, &gate, false, bin_cnt_snake, &roi, NULL);
  t_end = clock();

  struct point_t roi[2];
  roi[0].x = gate.x-gate.sz; roi[0].y = gate.y-gate.sz_left;
  roi[1].x = gate.x+gate.sz; roi[1].y = gate.y+gate.sz_left;
  //uint32_t avg = image_roi_mean(&d, roi);
  //printf("avg = %d\n", avg);
  printf("gate %d (%d,%d) %d %d %d\n", gate.q, gate.x, gate.y, gate.sz, gate.n_sides, gate.rot);

  for (int y = 0; y < d.h; y++) {
    for (int x = 0; x < d.w; x++) {
      output->at<uchar>(y, x) = ((uint8_t*)d.buf)[x + y*d.w];
    }
  }
}
