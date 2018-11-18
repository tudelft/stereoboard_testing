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

#include "camera_type.h"
#include "line_fit.h"
#include "stereo_math.h"

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
  //gate_set_color(30, 190, 90, 140, 160, 255, COLOR_SPACE_YUV); // red
  //gate_set_color(54,157,99,127,81,123, COLOR_SPACE_YUV); // green
  gate_set_color(50, 120, 60, 255, 60, 255, COLOR_SPACE_HSV); // green hsv
  //gate_set_color(160,255,75,125,129,157, COLOR_SPACE_YUV); // yellow
#endif
}

void test_line_fit(void){
  struct point_t points[128];
  float r2;
  for (uint16_t i = 0; i < 128; i++){
    points[i].x = i;
    points[i].y = 0.5 * points[i].x + 5; // + rand() % 2
    points[i].y = 0.1 * points[i].x * points[i].x + 0.5 * points[i].x + 5; // + rand() % 2
    points[i].y = i; // + rand() % 2
    points[i].x = 0.5 * points[i].y + 5;
  }

  float coefficient[3];
  int32_t coefficient_i[3];
  struct timeval tv_start, tv_end;
  int32_t slope, intercept;
  gettimeofday(&tv_start, NULL);
  for (uint32_t i; i < 100000; i++){
    r2 = line_fit_i(points, 128, 100, coefficient_i, 1);
  }
  gettimeofday(&tv_end, NULL);

  printf("took %f s, %d %d, %f\n", tv_end.tv_sec + (float)tv_end.tv_usec/1e6 - tv_start.tv_sec - (float)tv_start.tv_usec/1e6,
      slope, intercept, r2);

  gettimeofday(&tv_start, NULL);
  for (uint32_t i; i < 100000; i++){
    r2 = line_fit_f(points, 128, coefficient, 1);
  }
  gettimeofday(&tv_end, NULL);
  printf("took %f s, %f %f, %f\n", tv_end.tv_sec + (float)tv_end.tv_usec/1e6 - tv_start.tv_sec - (float)tv_start.tv_usec/1e6,
      coefficient[1], coefficient[0], r2);

  gettimeofday(&tv_start, NULL);
  for (uint32_t i; i < 100000; i++){
    r2 = line_fit_second_order_f(points, 128, coefficient, 1);
  }
  gettimeofday(&tv_end, NULL);
  printf("took %f s, %f %f %f, %f\n", tv_end.tv_sec + (float)tv_end.tv_usec/1e6 - tv_start.tv_sec - (float)tv_start.tv_usec/1e6,
      coefficient[2], coefficient[1], coefficient[0], r2);
}


int main(int argc, const char **argv)
{
  //test_line_fit();
  //return;

  initialise_gate_settings();

  //structures for images
  Mat image;
  uint8_t image_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * 2] = {0};

  Mat output(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0,0,0));
  Mat output_big(480, 640, CV_8UC3, Scalar(0,0,0));

  // open video capture
//  VideoCapture cap("/media/kirk/EUROGNC2013//test2_raw_images/color%1d.png");
  //VideoCapture cap("/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/test3_raw_images_flapping/%03d.png");
  //VideoCapture cap("/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/color_gates/color%1d.png");
  //VideoCapture cap("/media/kirk/EUROGNC2013/white_thick_line_green_obstacles/color%1d.png");
  VideoCapture cap("/media/kirk/EUROGNC2013/imav_2018_dataset_on_course/color%1d.png");

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
		if(counter < 530)
		  continue;

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
    uint8_t test = 4;
    if (test == 0){
      //struct timeval tv_start, tv_end;
      //gettimeofday(&tv_start, NULL);
      //for (int k = 0; k < 500; k ++)
      gate_color_detector(&input, &output);
      //gettimeofday(&tv_end, NULL);

      //printf("took %f s\n", tv_end.tv_sec + (float)tv_end.tv_usec/1e6 - tv_start.tv_sec - (float)tv_start.tv_usec/1e6);

      printf("%d %d %d\n", bin_cnt_snake[0], bin_cnt_snake[1], bin_cnt_snake[2]);
    } else if (test == 1) {
      struct roi_t segments[128];
      uint16_t num_found  = color_obstacle_detection(&input, &input, 54,157, 99,127, 81,123, 10, 0, segments, 128);
    } else if (test == 2) {
      float min_dist_btween_obstacles = 0.5f;  // m
      float flight_alt = 1.5;  // m
      float cam_angle = 0.785; //45 deg up from vertical
      uint16_t width_min = (uint16_t)ceilf(input.w*min_dist_btween_obstacles/(flight_alt*sinf(cam_angle+FOVY/2.f)*2.f*sinf(FOVX/2.f)));
      uint16_t width_max = (uint16_t)ceilf(input.w*min_dist_btween_obstacles/(flight_alt*sinf(cam_angle-FOVY/2.f)*2.f*sinf(FOVX/2.f)));
      printf("%d %d\n", width_min, width_max);
      struct point_t obstacle = color_obstacle_detection_with_keepout(&input, &input, 54,157, 99,127, 81,123, 10, 0, width_min, width_max);
      printf("centroid (%d %d) %f\n", obstacle.x, obstacle.y,flight_alt*cosf((((float)(obstacle.y - IMAGE_HEIGHT/2) * FOVY / IMAGE_HEIGHT) + cam_angle)));

    } else if (test == 3) {
      struct point_t points[256];
      uint32_t i = 0, j = 0;
      uint8_t *buf = (uint8_t*)input.buf;
      uint32_t x_sum = 0, y_sum = 0;
      while (i < 1024 && j < 256){
        uint16_t x = rand() % input.w;
        uint16_t y = rand() % input.h;
        if (buf[(x + input.w * y ) * 2 + 1] > 210){
          points[j].x = x;
          points[j].y = y;
          x_sum += x;
          y_sum += y;
          j++;
        }
        i++;
      }

      if (j){
        float coefficients[3];
        uint8_t dir = 1;
        float r2 = line_fit_second_order_f(points, j, coefficients, dir);

        struct point_t centroid = yuv_colorfilt_centroid(&input,NULL,180,255,0,255,0,255, 20, 0);

        image_show_points(&input, points, j);

        struct point_t line[128];
        uint32_t k = 0;
        if (dir){
          for (uint32_t y = 0; y < 96; y++){
            uint32_t x = (coefficients[2] * y + coefficients[1]) * y + coefficients[0];
            if (x > 0 && x < 128){
              line[k].x = x;
              line[k].y = y;
              k++;
            }
          }
        } else {
          for (uint32_t x = 0; x < 128; x++){
            uint32_t y = (coefficients[2] * x + coefficients[1]) * x + coefficients[0];
            if (y > 0 && y < 96){
              line[k].x = x;
              line[k].y = y;
              k++;
            }
          }
        }
        image_show_points_color(&input, line, k, yuv_red);
        printf("%f %f %f %f, (%d %d) (%d %d)\n", coefficients[0], coefficients[1], coefficients[2], r2, x_sum/j, y_sum/j, centroid.x, centroid.y);
      }
    } else if (test == 4){
      static float hoop_distances[5] = {0., 2.73, 5.33, 9.03, 11.11};
      static float hoop_diameters[5] = {1.3, 1.2, 0.93, 0.74, 0.48};
      static float x_loc = -3.;

      if (counter == 560 || counter == 600 || counter == 635)
       x_loc += 2.5f;

      gate_color_detector(&input, &output);
      if (gate.q > 14){
        float gate_ang = pix2angle(gate.sz, 0);
        float min_error = 100.f;
        uint16_t min_idx = 0;
        for (int32_t i = 0; i < 5; i++){
          float dis_obs = hoop_distances[i] - hoop_diameters[i]/(2*sinf(gate_ang));
          float obs_error = x_loc - dis_obs;
          if (fabsf(obs_error) < min_error){
            min_error = fabsf(obs_error);
            min_idx = i;
          }
          printf("%f %f %f\n", dis_obs, x_loc, obs_error);
        }
        if (min_error < 1.5f){
          x_loc += ((hoop_distances[min_idx] - hoop_diameters[min_idx]/(2*sinf(gate_ang))) - x_loc)/4.f;
          printf("%d, Likely gate %d, error %f, distance %f\n", gate.q, min_idx, min_error, x_loc);
        }
      }
    } else if (test == 5){
      // window
      static float window_width = 0.5f;
      static float distance = 2.f;
      gate_color_detector(&input, &output);
      if (gate.q > 14){
        float gate_ang = pix2angle(gate.sz, 0);
        float dist_obs = window_width/(2*sinf(gate_ang));
        float alpha = 0.1f;
        distance = distance*(1-alpha) + (dist_obs - distance)*alpha;
        printf("obs %f, est %f\n", dist_obs, distance);
      }
    } else if (test == 6){
      uint8_t sobel_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * 2] = {0};
      struct image_t sobel;
      sobel.buf = sobel_buffer;
      sobel.w = IMAGE_WIDTH;
      sobel.h = IMAGE_HEIGHT;
      sobel.buf_size = IMAGE_WIDTH*IMAGE_HEIGHT*2;
      sobel.type = IMAGE_YUV422;

      //image_2d_sobel(&input, &sobel);
      image_2d_sobel2(&input, &sobel);
      memcpy(input.buf, sobel.buf, input.buf_size);
    }

    yuv4222mat(&input, &output);


#endif

    resize(output, output_big, Size(), (float)output_big.cols/output.cols, (float)output_big.rows/output.rows);
    namedWindow( "after", WINDOW_AUTOSIZE);      // Create a window for display.
    imshow( "after", output_big);                   // Show our image inside it.

    waitKey(1);

    if (make_movie){
      // encode the frame into the videofile stream
      outputVideo.write(output_big);
    } else if (1){//gate.q > 30) {
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
  if (image->type == IMAGE_YUV422){
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
  } else if (image->type == IMAGE_GRAYSCALE){
    for (int y = 0; y < image->h; y++) {
      for (int x = 0; x < image->w; x++) {
        output->at<uchar>(y, x) = ((uint8_t*)image->buf)[idx];
        idx++;
      }
    }
    cvtColor(*output, *output, COLOR_GRAY2BGR);
  }

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
  memset(bin_cnt_snake, 0, sizeof(bin_cnt));  // reset the counters
  t_start = clock();
  snake_gate_detection(input, &gate, false, bin_cnt_snake, NULL, NULL);
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
