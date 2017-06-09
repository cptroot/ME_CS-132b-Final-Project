#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>


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

geometry_msgs::Twist get_next_move(
        std::vector<coordinate> path,
        geometry_msgs::Transform) {
  geometry_msgs::Twist result;

  return result;
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

  double current_height = 0.75;
  dstar = new Dstar(coordinate(65536 / 32 + 10, 65536 / 32 + 10), coordinate(65536 / 32 + 40, 65536 / 32 + 10), current_height);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    /*geometry_msgs::Twist msg;
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

    auto transform = transformStamped.transform;
    coordinate current_cell(transform.translation.x, transform.translation.y);
    auto path = dstar->navigate_map(current_cell);

    msg = get_next_move(path, transform);

    cmd_vel_publisher.publish(msg);*/

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
