#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <stdlib.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");

  // initialize the parameters
  //double x, y, z, w, nx, ny, nz;
  std::vector<double> path;  // [cx, cy, cz, radius]
  double cx, cy, radius;
  nh_private_.param("path", path, std::vector<double>(3,0));
  cx = path[0];
  cy = path[1];
  radius = path[2];

  ROS_INFO("Path Info [center_x, center_y, radius]: [%f, %f, %f]", cx, cy, radius);
  // transform euler angles to quaternions
  // quaternion type: [nx,ny,nz,w]
  //tf::Quaternion qt;
  //qt = tf::createQuaternionFromRPY(euler[0], euler[1], euler[2]);
  //ROS_INFO("transformed qt: [%f, %f, %f, %f]", qt[0], qt[1], qt[2], qt[3]);


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  const float DEG_2_RAD = M_PI / 180.0;
  double theta = 0;
  move_base_msgs::MoveBaseGoal goal;

  // while the action server is connected
  int count = 0;
  while (ac.isServerConnected () && ros::ok()) {

    //update
    switch (count) {
      case 0: {
        goal.target_pose.pose.position.x = cx + radius;
        goal.target_pose.pose.position.y = cy;
        goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI/2);
        break;
      }
      case 1: {
        goal.target_pose.pose.position.x = cx;
        goal.target_pose.pose.position.y = cy + radius;
        goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
        break;
      }
      case 2: {
        goal.target_pose.pose.position.x = cx - radius;
        goal.target_pose.pose.position.y = cy;
        goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI*3/2);
        break;
      }
      case 3: {
        goal.target_pose.pose.position.x = cx;
        goal.target_pose.pose.position.y = cy - radius;
        goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        break;
      }
      default: {
        goal.target_pose.pose.position.x = 0;
        goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        break;
      }
    }

    count = (count+1) % 4;

    theta = fmod((theta+0.02), (2*M_PI));

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();


    // publish the waypoint
    ROS_INFO("Publishing waypoint[%d] : [%f, %f]", count, goal.target_pose.pose.position.x,
    goal.target_pose.pose.position.y);

    ac.sendGoal(goal);

    // wait until it reaches the goal
    ac.waitForResult();
  }

  return 0;
}
