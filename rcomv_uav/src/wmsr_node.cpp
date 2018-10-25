

#include "wmsr_node.h"

// Constructor
WMSRNode::WMSRNode()
  :nh_private_("~")
  {
    // Initialize variables assigned in the launch file
    nh_private_.param<int>("n", n, 15);
    nh_private_.param<int>("k", k, 7);

    nh_private_.param<int>("idx", idx, 1);
    nh_private_.param<int>("role", role, 2);

    nh_private_.param<int>("F", F, 0);
    //nh_private_.param<std::vector<std::vector<int>>>("L", L,
    //          std::vector<std::vector<int>>(15, std::vector<int>(15,0)) );

    nh_private_.param<int>("x", x0, 0);
    nh_private_.param<int>("y", y0, 0);
    nh_private_.param<int>("z", z0, 0);


    // Initialize msgs
    own_states.header.stamp = ros::Time::now();
    own_states.point.x = x0;
    own_states.point.y = y0;
    own_states.point.z = z0;
    mali_states.header.stamp = ros::Time::now();
    mali_states.point.x = x0;
    mali_states.point.y = y0;
    mali_states.point.z = z0;



    // Publisher: reference
    //std::string pub_topic = "WMSR" + std::to_string(idx) + "/ref";
    std::string pub_topic = "WMSR/ref";     // pub topic is relative to the node namespace
    ref_pub = nh.advertise<ref_msgs>(pub_topic, 10);
    // Publisher: output
    output_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
              mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    // Publisher Timer
    // frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::ref_pubCallback, this);
    out_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::out_pubCallback, this);

    // Subscribers: (and msgs list to hold the msgs from subscibed topics)
    for (int i = 1; i <= k; i++) {
      // Initialize msgs list that holds msgs from neighbor agents
      ref_msgs ref_point;
      ref_lists.push_back(ref_point);

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx -i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/uav" + std::to_string(sub_idx) + "/WMSR/ref";

      //ref_subs.push_back(nh.subscribe(sub_topic, 10,
      //                    &WMSRNode::subCallback, this));
      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 10,
                         boost::bind(&WMSRNode::subCallback, this, _1, i-1)) );

      ROS_INFO("sub_idx at: [%d] with topic name: ", sub_idx);
    }

  }


// Destructor
WMSRNode::~WMSRNode()
{
  ros::shutdown();
}


// Subscriber Callback Function
void WMSRNode::subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx)
{
  ref_lists[list_idx].header = msgs->header;
  ref_lists[list_idx].point = msgs->point;
}

// Reference Publisher Callback
void WMSRNode::ref_pubCallback(const ros::TimerEvent& event)
{
  // Role: 1 = Malicious, 2 = Normal, 3 = Leader
  // Leader node
  // Static Formation: node states are static
  if (role == 3) {
      own_states.header.stamp = ros::Time::now();
      ref_pub.publish(own_states);    // publish reference states to other WMSR node
  }
  // Normal node
  // implement WMSR algorithm to update node states
  else if (role == 2) {
    own_states = WMSRNode::WMSRAlgorithm(ref_lists);
    ref_pub.publish(own_states);    // publish reference states to other WMSR node
  }
  // Malicious node
  // implement WMSR algorithm to calculate a reference states
  // If it's cyber attack, update node states to reference states, but publish a malicious state
  // If it's physical attack, update node states to a malicious states
  else {
    own_states = WMSRNode::WMSRAlgorithm(ref_lists);

    mali_states.header.stamp = ros::Time::now();
    mali_states.point.x += 0.1;
    mali_states.point.y += 0.1;
    mali_states.point.z = std::max(0.0, mali_states.point.z-1);

    ref_pub.publish(mali_states);    // publish reference states to other WMSR node
  }

}

// Output Publisher Callback
void WMSRNode::out_pubCallback(const ros::TimerEvent& event)
{
  // calculate the node states in the formation
  WMSRNode::Formation();

  //
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

  double x, y, z;
  //x = own_formation_states.point.x;
  //y = own_formation_states.point.y;
  // only consider 1d motion
  x = x0;
  y = y0;
  z = own_formation_states.point.z;
  const float DEG_2_RAD = M_PI / 180.0;

  Eigen::Vector3d waypoint_position(x, y, z);
  double desired_yaw = 0 * DEG_2_RAD;

  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
    waypoint_position, desired_yaw, &trajectory_msg);

  //
   output_pub.publish(trajectory_msg);
}

// Helper Function::
ref_msgs WMSRNode::WMSRAlgorithm(const std::vector<ref_msgs> &list)
{
  ref_msgs ref_states;

  std::vector<double> listx(k,0);
  std::vector<double> listy(k,0);
  std::vector<double> listz(k,0);

  double wx, wy, wz; // uniform weights

  double wav_x = 0, wav_y = 0, wav_z = 0; // weighted averages

  double x, y, z;
  x = own_states.point.x;
  y = own_states.point.y;
  z = own_states.point.z;

  // scan the data
  for (int i = 0; i < k; i++) {
    listx[i] = list[i].point.x;
    listy[i] = list[i].point.y;
    listz[i] = list[i].point.z;
  }

  //remove outliers
  wx = FilterOutlier(listx, k, x, F);
  wy = FilterOutlier(listy, k, y, F);
  wz = FilterOutlier(listz, k, z, F);

  // update node states
  // note: size increases by 1 due to the added node value
  for (int j = 0; j < k+1; j++) {
    wav_x += wx * listx[j];
    wav_y += wy * listy[j];
    wav_z += wz * listz[j];
  }

  ref_states.header.stamp = ros::Time::now();
  ref_states.point.x = wav_x;
  ref_states.point.y = wav_y;
  ref_states.point.z = wav_z;

  return ref_states;
}

// Helper Function: Create formation
void WMSRNode::Formation()
{
  own_formation_states.header.stamp = ros::Time::now();

  own_formation_states.point = own_states.point;
}

// Helper Function: remove outlier entries
double FilterOutlier(std::vector<double> &list, const int k, const double own_state, const int F)
{
  int num_h = 0, num_l = 0;
  int valid_size = k;

  // record the number of values
  // that are greater or less than the current node value
  for (int i = 0; i < k; i++) {
    if (list[i] > own_state)   num_h++;
    if (list[i] < own_state)   num_l++;
  }

  // sort the list in ascending order
  std::sort(list.begin(), list.end());

  // if there are F or more values greater (less than) the state value, remove
  // the first F states greater (less than) the state value.
  // Otherwise, remove all states greater (less than) the state value.
  if (num_h > F) {
    valid_size -= F;
    for (int j = k-1; j >= k-F; j--)
      list[j] = 0;
  }
  else {
    valid_size -= num_h;
    for (int j = k-1; j >= k-num_h; j--)
      list[j] = 0;
  }

  if (num_l > F) {
    valid_size -= F;
    for (int j = 0; j <= F-1; j++)
      list[j] = 0;
  }
  else {
    valid_size -= num_l;
    for (int j = 0; j <= num_l-1; j++)
      list[j] = 0;
  }

  // append the current node value to the remaining list
  list.push_back(own_state);
  valid_size++;

  return 1.0 / double(valid_size);
}


// main function
int main(int argc, char **argv) {

  // Initialize ros
  ros::init(argc, argv, "WMSR_Node");

  // Create an object of WMSRNode that handles everything
  WMSRNode WMSR_Node;

  ros::spin();

  return 0;
}
