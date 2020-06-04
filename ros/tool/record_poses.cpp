#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

using namespace std;

class PoseRecorder
{
public:
  PoseRecorder(ros::NodeHandle& nh, string outputFilePath) : nh_(nh)
  {
    // initialize output file
    outputFile_.open(outputFilePath);
    if (!outputFile_.is_open())
      cerr << "Invalid path of output file!" << endl;
    else
      outputFile_ << "# timestamp x y z roll pitch yaw" << endl;

    // create subscriber
    poseSub_ = nh.subscribe("/orb_slam2_mono/pose", 100,
                            &PoseRecorder::save_pose, this);
  }

private:
  void save_pose(geometry_msgs::PoseStampedConstPtr msg)
  {
    // get stamp in us
    long stamp = static_cast<long>(msg->header.stamp.toNSec() / 1e3);

    // get xyz
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // get RPY
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    outputFile_ << stamp << ' ' << std::fixed << setprecision(6) << x << ' '
                << y << ' ' << z << ' ' << roll << ' ' << pitch << ' ' << yaw
                << endl;

    cout << "Pose recorded!" << endl;
  }

  ros::NodeHandle nh_;
  ofstream outputFile_;
  ros::Subscriber poseSub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_recorder");
  ros::NodeHandle nh;

  if (argc != 2)
  {
    cerr << "Usage: rosrun orb_slam_2_ros pose_recorder /path/to/poses_file.txt"
         << endl;
    return 1;
  }

  // create recorder
  PoseRecorder pr(nh, argv[1]);

  // wait for procesing pose messages
  ros::spin();

  return 0;
}
