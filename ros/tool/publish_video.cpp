#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool isFileNotOpenedOrEmpty(ifstream& dataFile)
{
  return !dataFile.is_open() ||
         dataFile.peek() == std::ifstream::traits_type::eof();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_publisher");
  ros::NodeHandle nh;

  if (argc < 3)
  {
    cerr << "Usage: rosrun orb_slam_2_ros video_publisher camera.timestamps "
            "video_file [mask_image](optional)" << endl;
    return 1;
  }

  // open timestamps file
  ifstream stampFile(argv[1]);
  if (isFileNotOpenedOrEmpty(stampFile))
  {
    cerr << "Input data file(s) are not valid!" << endl;
    return 1;
  }

  // open video file
  VideoCapture cap(argv[2]);
  if (!cap.isOpened())
  {
    cerr << "Input video file is not valid!" << endl;
    return 1;
  }

  // open mask image
  Mat mask;

  if (argc = 4)
  {
    mask = imread(argv[3], IMREAD_GRAYSCALE);
  }

  // create publisher
  image_transport::ImageTransport it(nh);
  image_transport::Publisher video_pub = it.advertise("/camera/image_raw", 10);

  // publish
  cout << "Start to publish..." << endl;

  int frame_num = 0;
  //  int ignore_num = 945; // 945 to 1300
  //  int ignore_num = 1640; // 1640 to 1840
  //  int ignore_num = 2020; // 2020 to 2600
  //  int ignore_num = 4775; // 4775 to 4955
  //  int ignore_num = 8700; // 8700 to 8950
  int ignore_num = 3320; // 3320 to 3510
  cout << "ignore the first " << ignore_num << " frames" << endl;

  double scale = 0.7;
  cout << "scale factor: " << scale << endl;

  string line;

  while (ros::ok() && getline(stampFile, line))
  {
    std::stringstream linestream(line);
    int index;
    uint64_t stamp;
    linestream >> index >> stamp;

    Mat image_raw;
    cap >> image_raw;

    if (image_raw.empty())
      break;

    if (frame_num < ignore_num)
    {
      ++frame_num;
      continue;
    }

    ros::Time t;
    t.fromNSec(stamp * 1e3);

    std_msgs::Header header;
    header.frame_id = "camera_link";
    header.seq = frame_num;
    header.stamp = t;

    // apply mask if exist
    Mat image_masked;
    if (!mask.empty())
      image_raw.copyTo(image_masked, mask);
    else
      image_masked = image_raw;

    // resize
    int width_scaled = image_masked.cols * scale;
    int height_scaled = image_masked.rows * scale;

    Mat image;
    resize(image_masked, image, Size(width_scaled, height_scaled));

    sensor_msgs::ImagePtr image_msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    video_pub.publish(image_msg);

    cout << "publishing video frame #" << frame_num << endl;

    ++frame_num;

    ros::Duration(1.0).sleep();
  }

  cout << "Video publisher finished!" << endl;

  return 0;
}
