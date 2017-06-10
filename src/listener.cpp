#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>
#include <math.h>

#include "Dstar.hpp"

Dstar *dstar;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void octomapCallback(const octomap_msgs::Octomap& msg)
{
  ROS_INFO("I heard: [%s]", msg.id.c_str());

  octomap::OcTree * tree = (octomap::OcTree *)octomap_msgs::msgToMap(msg);

  ROS_INFO("Tree size: %lu", tree->size());

  ROS_INFO("root node size: %f", tree->getNodeSize(0));

  dstar->change_map(tree);
}

struct nav_inst {
    float linear_x;
    float linear_y;
    float angular_z;
    
    nav_inst() {}
    nav_inst(float x, float y, float z) : linear_x(x), linear_y(y), angular_z(z) {}
};

nav_inst navigation(coordinate curr, coordinate next, double angle_);

geometry_msgs::Twist get_next_move(
        std::vector<coordinate> path,
        coordinate curr_cell,
        geometry_msgs::Transform tf) {
  geometry_msgs::Twist result;

  if (path.size() > 1) {
        geometry_msgs::Quaternion quat = tf.rotation;
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        coordinate next = path[1];

        auto navigation_instruction = navigation(curr_cell, next, yaw);

        result.linear.x = navigation_instruction.linear_x;
        result.linear.y = navigation_instruction.linear_y;
        result.angular.z = navigation_instruction.angular_z;

        return result;
  } else {
      return result;
  }
}

nav_inst navigation(coordinate curr, coordinate next, double angle_) {
    const double PI = 3.141592653589793;
    double new_angle_ = atan2(next.y - curr.y, next.x - next.y);
    double new_angle = fmod(new_angle_, (2 * PI));
    double angle = fmod(angle_, (2 * PI));
    
    if (fabs(angle - new_angle) > 0.5) {
        if (angle > new_angle) {
            return nav_inst(0, 0, 0.5);
        }
        else {
            return nav_inst(0, 0, -0.5);
        }
    }
    else if ((fabs(curr.x - next.x) > 0.5) && (fabs(curr.y - next.y) > 0,5)) {
        return nav_inst(0.1, 0, 0);
    }
    else{
        return nav_inst(0,0,0);
    }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "dstar");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("octomap", 10, octomapCallback);
  ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ROS_INFO("Created publisher and subscriber.");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::Transform startTransform;
  bool foundStart = false;

  while (!foundStart) {
    try {
        auto transformStamped =
            tfBuffer.lookupTransform("map", "cam0",
            ros::Time(0));
        startTransform = transformStamped.transform;
        foundStart = true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }
  }

  double current_height = startTransform.translation.z;
  coordinate start = coordinate(std::round(startTransform.translation.x * 10), std::round(startTransform.translation.y * 10));
  coordinate goal = coordinate(start.x + 10, start.y);
  dstar = new Dstar(coordinate(65536 / 32 + start.x, 65536 / 32 + start.y), coordinate(65536 / 32 + goal.x, 65536 / 32 + goal.y), current_height);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped =
            tfBuffer.lookupTransform("world", "base_link",
            ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }

    auto tf = transformStamped.transform;
    coordinate curr_cell(std::round(tf.translation.x * 10) + 65536 / 32,
          std::round(tf.translation.y * 10) + 65536 / 32);
    auto path = dstar->navigate_map(curr_cell);

    msg = get_next_move(path, curr_cell, tf);

    cmd_vel_publisher.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
