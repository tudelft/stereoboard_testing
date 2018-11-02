#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
//Include header file of stereoboard code
#include "stereoboard/gate_detection.h"
#include "stereoboard/main_parameters.h"
#include "gnuplot_i.hpp"

#include "cv/image.h"
#include "cv/image.h"

using namespace std;
using namespace cv;

void mat2yuv422(Mat* image, uint8_t* image_buffer);
void yuv4222mat(struct image_t* image, Mat* output);
void generate_mux_image(Mat* image_l, Mat* image_r, uint8_t* image_buffer_left, uint8_t* image_buffer_right, uint8_t* image_buffer);
void gate_color_detector(struct image_t* input, Mat* grad);
void gate_gray_detector(struct image_t* input, Mat* grad);
void gate_gradient_detector(struct image_t* input, Mat* grad);


uint16_t bin_cnt[3] = {0};
uint16_t bin_cnt_snake[3] = {0};
struct roi_t roi;

struct gate_t gate;

bool make_movie = false;
const string video_name = "./gate_video.avi";
clock_t t_start, t_end;

uint32_t positive = 0, negative = 0;

#define COLOR    0
#define GRAY     1
#define GRADIENT 2

#define IMAGE_TYPE COLOR

void initialise_gate_settings(void)
{
  //struct roi_t ROI = {.tl={0,0}, .tr={img->w,0}, .br={img->w,img->h}, .bl={0,img->h}};

#if IMAGE_TYPE == GRADIENT
  gate_set_intensity(50, 255); // nus meeting room
#elif IMAGE_TYPE == GRAY
  gate_set_intensity(5, 108); // nus meeting room
#else
  //gate_set_color(30, 190, 90, 140, 160, 255); // red
  gate_set_color(54,157,99,127,81,123);
#endif
}

int main(int argc, const char **argv)
{
  initialise_gate_settings();

	//structures for images
	Mat image;
	uint8_t image_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * 2] = {0};

	Mat output(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0,0,0));
	Mat output_big(480, 640, CV_8UC3, Scalar(0,0,0));

	// open video capture
	VideoCapture cap("/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/color_gates/color%1d.png");

	if (!cap.isOpened()) {
	  printf("Couldn't open images\n");
	  return -1;
	}

	int counter = 0;

	cap >> image;
  counter++;

  VideoWriter outputVideo;                                        // Open the output
  if(make_movie){
    int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
    outputVideo.open(video_name, codec, 6., output_big.size(), true);

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
	  printf("frame: %d\n", counter);
    counter++;
    cap >> image;

		if (image.empty()) {
		  printf("empty image\n");
			break;
		}

		// Crop out the separate left and right images
/*    if (image.channels() == 3) {
/      cvtColor(image, image, COLOR_BGR2GRAY);
    }*/

    namedWindow( "image", WINDOW_AUTOSIZE );  // Create a window for display.
    imshow( "image", image );               // Show our image inside it.

    resize(image, image, Size(), (float)IMAGE_WIDTH/image.cols, (float)IMAGE_HEIGHT/image.rows);
    mat2yuv422(&image, image_buffer);

    struct image_t input;
    input.buf = image_buffer;
    input.w = IMAGE_WIDTH;
    input.h = IMAGE_HEIGHT;
    input.buf_size = IMAGE_WIDTH*IMAGE_HEIGHT*2;
    input.type = IMAGE_YUV422;

#if IMAGE_TYPE == GRADIENT
    input.type = IMAGE_GRAYSCALE;
    gate_gradient_detector(&input, &output);
#elif IMAGE_TYPE == GRAY
    input.type = IMAGE_GRAYSCALE;
    gate_gray_detector(&input, & output);
#else
    input.type = IMAGE_YUV422;
    //gate_color_detector(&input, &output);
    struct point_t centroid;
    //centroid = yuv_colorfilt_centroid(&input, &input, 54,157, 99,127, 81,123, 20, 0);
    struct roi_t segments[128];
    centroid = color_obstacle_detection(&input, &input, 54,157, 99,127, 81,123, 10, 0, segments, 128);
    printf("centroid (%d %d)\n", centroid.x,centroid.y);
    yuv4222mat(&input,&output);
    getchar();
#endif

    resize(output, output_big, Size(), (float)output_big.cols/output.cols, (float)output_big.rows/output.rows);
    namedWindow( "after", WINDOW_AUTOSIZE);      // Create a window for display.
    imshow( "after", output_big);                   // Show our image inside it.

    waitKey(1);

    if (make_movie){
      // encode the frame into the videofile stream
      outputVideo.write(output_big);
    } else if (gate.q > 30) {
      printf("frame: %d %f ms\n", counter, ((double) (t_end - t_start)) * 1000 / CLOCKS_PER_SEC);
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

void mat2yuv422(Mat* image, uint8_t* image_buffer)
{
  cvtColor(*image, *image, COLOR_BGR2YUV);
  // Put image values in array, just like in the stereoboard
  int x, y, idx = 0;

  for (y = 0; y < image->rows; y++) {
    for (x = 0; x < image->cols; x++)
    {
      if(idx % 4 == 0){
        image_buffer[idx] = (uint8_t)image->at<Vec3b>(y, x)[1];
      } else {
        image_buffer[idx] = (uint8_t)image->at<Vec3b>(y, x)[2];
      }
      image_buffer[idx+1] = (uint8_t)image->at<Vec3b>(y, x)[0];
      idx += 2;
    }
  }
}

void yuv4222mat(struct image_t* image, Mat* output)
{
  uint32_t idx = 0;
  for (int y = 0; y < image->h; y++) {
    for (int x = 0; x < image->w; x++) {
      output->at<Vec3b>(y, x)[0] = ((uint8_t*)image->buf)[idx+1];
      if (idx % 4 == 0){
        output->at<Vec3b>(y, x)[1] = ((uint8_t*)image->buf)[idx];
        output->at<Vec3b>(y, x)[2] = ((uint8_t*)image->buf)[idx+2];
      } else {
        output->at<Vec3b>(y, x)[1] = ((uint8_t*)image->buf)[idx+2];
        output->at<Vec3b>(y, x)[2] = ((uint8_t*)image->buf)[idx];
      }
      idx += 2;
    }
  }
  cvtColor(*output, *output, COLOR_YUV2BGR);
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

void gate_color_detector(struct image_t* input, Mat* output){
  t_start = clock();
  snake_gate_detection(input, &gate, false, NULL, NULL, NULL);
  t_end = clock();

  yuv4222mat(input, output);
}

void gate_gray_detector(struct image_t* input, Mat* output)
{
  memset(bin_cnt_snake, 0, sizeof(bin_cnt));  // reset the counters

  t_start = clock();
  snake_gate_detection(input, &gate, false, bin_cnt_snake, &roi, NULL);
  t_end = clock();

//  struct point_t roi[2];
//  roi[0].x = gate.x-gate.sz; roi[0].y = gate.y-gate.sz_left;
//  roi[1].x = gate.x+gate.sz; roi[1].y = gate.y+gate.sz_left;
  //uint32_t avg = image_roi_mean(&d, roi);
  //printf("avg = %d\n", avg);
  printf("gate %d (%d,%d) %d %d %f\n", gate.q, gate.x, gate.y, gate.sz, gate.n_sides, gate.rot);

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

//  struct point_t roi[2];
//  roi[0].x = gate.x-gate.sz; roi[0].y = gate.y-gate.sz_left;
//  roi[1].x = gate.x+gate.sz; roi[1].y = gate.y+gate.sz_left;
  //uint32_t avg = image_roi_mean(&d, roi);
  //printf("avg = %d\n", avg);
  printf("gate %d (%d,%d) %d %d %f\n", gate.q, gate.x, gate.y, gate.sz, gate.n_sides, gate.rot);

  for (int y = 0; y < d.h; y++) {
    for (int x = 0; x < d.w; x++) {
      output->at<uchar>(y, x) = ((uint8_t*)d.buf)[x + y*d.w];
    }
  }
}
