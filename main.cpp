#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <argp.h>
#include <ctype.h>
#include "gnuplot_i.hpp"

using namespace std;
using namespace cv;

//Include header file of stereoboard code
#include "stereoboard/edgeflow.h"
#include "stereoboard/camera_type.h"

const char *argp_program_version = "v0.1";
const char *argp_program_bug_address = "https://github.com/tudelft/stereoboard_testing";
static char doc[] = "This is used to test functions in the stereoboard project.";
static char args_doc[] = "'database' 'take', e.g ./testing 1 3\n -d 'database' -t 'take' -i -p, e.g ./testing -d 1 -t 3 -i -p";
static struct argp_option options[] = {
  { "show_image", 'i', 0, 0, "Show image frame.",0},
  { "show_plot", 'p', 0, 0, "Show output of edgeflow parameters.",0},
  { "database", 'd', 1, 0, "Database number.",0},
  { "take", 't', 1, 0, "Dataset number.",0},
  { 0 }
};

struct args_t {
  bool show_image, show_plot;
  char *database;
  char *take;
};

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
  struct args_t *args = (struct args_t *)state->input;
  switch (key) {
    case 'i': args->show_image = true; break;
    case 'p': args->show_plot = true; break;
    case 'd': args->database = arg; break;
    case 't': args->take = arg; break;
    case ARGP_KEY_ARG:
      return 0;
    case ARGP_KEY_END:
      if (args->database == NULL || args->take == NULL)
      {
        cout<<"Please provide the database and take number"<<endl;
        argp_usage(state);
      }
      break;
    default:
      return ARGP_ERR_UNKNOWN;
  }
  return 0;
}

static struct argp argp = { options, parse_opt, args_doc, doc, 0, 0, 0 };


#define FOVX 1.001819   // 57.4deg = 1.001819 rad
#define FOVY 0.776672   // 44.5deg = 0.776672 rad

// initialize structures for plotting (can be multiple if you want)
Gnuplot g1("lines");
Gnuplot g2("lines");
Gnuplot g3("lines");
Gnuplot g4("lines");
Gnuplot g5("lines");


void plot_line_gnu(double A, double B, uint16_t size, Gnuplot *g, bool hold_on, string title);
void plot_gnu(int32_t *array, uint16_t size, Gnuplot *g, bool hold_on, string title);

int main(int argc, char *argv[])
{
  struct args_t args;

  args.show_image = false;
  args.show_plot = false;
  args.database = NULL;
  args.take = NULL;

  //check if too many arguments are passed
  if (argc == 3 && isdigit(argv[1][0]) && isdigit(argv[1][0])) {
    args.database = argv[1];
    args.take = argv[2];
  } else {
    argp_parse(&argp, argc, argv, 0, 0, &args);
  }

  cout << "Evaluating stereoboard dataset " << args.database << " with take " << args.take << endl;

  string configuration_board = "forward_camera";
  uint32_t frame_time = 0;

  // Find Directories
  stringstream file_directory_images;
  stringstream file_directory_timing;
  stringstream file_directory_calibration;
  stringstream file_directory_results;

  file_directory_images << "stereoboard_database/database_stereoboard_" << args.database << "/" <<
      configuration_board << "/take" << args.take << "/%1d.bmp";
  file_directory_timing << "stereoboard_database/database_stereoboard_" << args.database << "/" <<
      configuration_board << "/take" << args.take << "/timing.dat";
  file_directory_calibration << "stereoboard_database/database_stereoboard_" << args.database <<
      "/calibration_data.txt";
  file_directory_results << "stereoboard_database/database_stereoboard_" << args.database << "/" <<
      configuration_board << "/take" << args.take << "/result_stereo.csv";

  //initialize for edgeflow
  edgeflow_init(128, 94, 0);

  //Uncomment if you want to try an old version of the code
  //edgeflow_init(128, 94, 0);

  //Open files needed for testing
  VideoCapture cap; cap.open(file_directory_images.str());    //image location
  ofstream output; output.open(file_directory_results.str());   // result file
  fstream calibration_file(file_directory_calibration.str(), ios_base::in);
  calibration_file >> edgeflow_params.stereo_shift; // calibration data of that file
  fstream timing_file(file_directory_timing.str(), ios_base::in);

  //Make vector with timing data
  double num;
  string line, temp_str_c1, temp_str_c2;
  vector<double> timing;
  while (getline(timing_file,line))
  {
    istringstream ss(line);
    ss>>temp_str_c1; // read first column
    ss>>temp_str_c2; // read second column
    if(atof(temp_str_c2.c_str())==1.0) // If second column is not 1, then there is no accommodating image
    {
      num = (double)atof(temp_str_c1.c_str());
      timing.push_back(num);
    }
  }

  //OPENCV structures to read out images
  Rect ROI_right(0, 0, 128, 94); //Note that in the database, image left and right is reverted!
  Rect ROI_left(128, 0, 128, 94);
  Mat image_left, image_left_gray;
  Mat image_right, image_right_gray;
  Mat image;
  uint8_t image_buffer[128 * 94 * 2];
  uint8_t image_buffer_left[128 * 94 ];
  uint8_t image_buffer_right[128 * 94 ];

  Mat prev_image;
  char prev_image_file[255];
  struct image_t current_img;
  current_img.buf = (void*)image_buffer;
  current_img.w = IMAGE_WIDTH;
  current_img.h = IMAGE_HEIGHT;
  current_img.buf_size = 2*IMAGE_WIDTH*IMAGE_HEIGHT;
  current_img.pprz_ts = 0;

  if (!cap.isOpened()) { return -1; }

  //start loop while images are read
  int counter = 0;
  for (;;) {
    counter++;
    cap >> image;

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
    int x, y, idx, idx2;
    for (y = 0; y < image_left_gray.rows; y++) {
      for (x = 0; x < image_left_gray.cols; x++) {
        idx2 = image_left_gray.cols * y + x;

        image_buffer_left[idx2] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer_right[idx2] = (uint8_t)image_right_gray.at<uchar>(y, x);

#if (CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux)
        idx = 2 * (image_left_gray.cols * y + x);
        image_buffer[idx] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer[idx + 1] = (uint8_t)image_right_gray.at<uchar>(y, x);
#elif (CAMERA_CPLD_STEREO == camera_cpld_stereo_linemux)
        idx = 2 * image_left_gray.cols * y + x;
        image_buffer[idx] = (uint8_t)image_left_gray.at<uchar>(y, x);
        image_buffer[idx + image_left_gray.cols] = (uint8_t)image_right_gray.at<uchar>(y, x);
#endif
      }
    }

    //calculate edgeflow
    current_img.pprz_ts = (uint32_t)(timing.at(counter) * 1e6);
    edgeflow_total(&current_img, 0);

    //If you want to try an old version of the code
    //int16_t *stereocam_data;
    //edgeflow_total(image_buffer, frame_time,stereocam_data,0);


    // Plot results
    if(args.show_plot)
    {
      printf("frame: %d\n", counter);
      printf("vel: %d %d %d\n", edgeflow.vel.x, edgeflow.vel.y, edgeflow.vel.z);
      printf("avg_disp %d avg_dist %d\n", edgeflow.avg_disp, edgeflow.avg_dist);

      plot_gnu(edgeflow.disp.stereo, 128, &g1, false, "displacement.stereo");
      plot_gnu(edgeflow.disp.x, 128, &g1, true, "displacement.x");

      plot_gnu(edgeflow.edge_hist[edgeflow.current_frame_nr].x, 128, &g2, false, "edgehist");
      plot_gnu(edgeflow.edge_hist_right, 128, &g2, true, "edgehist_right");

      plot_gnu((int32_t *)edgeflow.disp.confidence_x, 128, &g3, false, "confidence x");
      plot_gnu((int32_t *)edgeflow.disp.confidence_stereo, 128, &g3, true, "confidence stereo");

      plot_gnu(edgeflow.disp.y, 96, &g4, false, "displacement.y");

      plot_gnu((int32_t *)edgeflow.disp.confidence_y, 96, &g5, false, "confidence y");

      if (!args.show_image)
      {
        std::cin.get();
      }
    }

    if (args.show_image)
    {
      namedWindow("Display window", WINDOW_OPENGL);  // Create a window for display.
      imshow("Display window", image_left_gray);
      waitKey(1);
      std::cin.get();
    }

    //calculate mean displacement
    int mean_disp_x_temp = 0;
    int mean_disp_stereo_temp = 0;

    for (x = 0; x < 128; x++) {
      mean_disp_x_temp += edgeflow.disp.x[x];
      mean_disp_stereo_temp += edgeflow.disp.stereo[x];
    }
    int mean_disp_x = 100 * mean_disp_x_temp / 128;
    int mean_disp_stereo = 100 * mean_disp_stereo_temp / 128;

    static struct vec3_t tot_dist = {0, 0, 0},prev_dist = {0, 0, 0};
    if (edgeflow_snapshot.new_keyframe)
    {
      prev_dist.x = tot_dist.x;
      prev_dist.y = tot_dist.y;
      prev_dist.z = tot_dist.z;
    }

    tot_dist.x = prev_dist.x + edgeflow_snapshot.dist.x;
    tot_dist.y = prev_dist.y + edgeflow_snapshot.dist.y;
    tot_dist.z = prev_dist.z + edgeflow_snapshot.dist.z;

    //Save data on output.cvs
    //TODO: also enter groundtruth data
    output << edgeflow.vel.x << "," << edgeflow.vel.y << "," << edgeflow.vel.z <<
        "," << edgeflow.vel_x_stereo_avoid_pixelwise << "," << edgeflow.vel_z_stereo_avoid_pixelwise
        << "," << edgeflow.avg_dist << "," << tot_dist.x << "," << tot_dist.y << "," << tot_dist.z << endl;
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

  if (hold_on == false) {
    g->reset_plot();
  }

  g->set_style("lines").plot_xy(X, Y, title);

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
    Y.push_back((double)(A * (x - size / 2) + B));
  }

  if (hold_on == false) {
    g->reset_plot();
  }

  g->set_style("lines").plot_xy(X, Y, title);

  if (plot_counter > 40) { //TODO: get rid of those irritating warnings...
    g->remove_tmpfiles();
    plot_counter = 0;
  } else {
    plot_counter++;
  }

}
