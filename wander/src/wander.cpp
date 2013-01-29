#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <numeric>

using namespace std;

geometry_msgs::Twist *
wander(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    assert(msg->ranges.size() >= 30);
    int angle;
    int halt = 0;
    int mid = msg->ranges.size() / 2;
    geometry_msgs::Twist * cmd = new geometry_msgs::Twist();
    // halt if an object is less than 2m in a 30deg angle
    for (angle = mid - 15; angle < mid + 15; angle++) {
        if (msg->ranges[angle] < 2) {
            halt = 1;
            break;
        }
    }
    if (halt != 0) {
        double midL, midR;
        midL = std::accumulate(msg->ranges.begin(), msg->ranges.end()-mid, 0);
        midR = std::accumulate(msg->ranges.begin()+mid, msg->ranges.end(), 0);
        // we go to the highest-range side scanned
        if (midL < midR) {
            cmd->angular.z = -1.0;
        } else {
            cmd->angular.z = +1.0;
        }
    } else {
        cmd->linear.x = 1.0;
    }

    return cmd;
}

ros::Publisher topic;

void handle_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    geometry_msgs::Twist * cmd = wander(msg);
    topic.publish(*cmd);
    delete(cmd);
}

int main(int argc, char **argv)
{
    // ./bin/wander cmd:=/robot/motion laser:=/robot/sick
    ros::init(argc, argv, "wander");
    ROS_INFO("wander roscpp initilized");
    ros::NodeHandle n;

    topic = n.advertise<geometry_msgs::Twist>("cmd", 1000);
    ros::Subscriber sub = n.subscribe("laser", 1000, handle_lidar);
    ros::spin();
    return 0;
}
