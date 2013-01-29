#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <ocl/Component.hpp>
#include <sensor_msgs/LaserScan.h>
#include <numeric>

using namespace std;
using namespace RTT;

class Wander : public RTT::TaskContext {
public:
    Wander(const std::string& name):
        TaskContext(name),
        inport("laser_in"),
        outport("twist_out")
    {
        ports()->addPort(inport);
        ports()->addPort(outport);
    }
    ~Wander() {}
private:
    InputPort<sensor_msgs::LaserScan> inport;
    OutputPort<geometry_msgs::Twist> outport;
    void wander(const sensor_msgs::LaserScan::ConstPtr& msg);
    void updateHook() {
        sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan());
        if (NewData == inport.read(*msg)) {
            wander(msg);
        }
    }
};

void Wander::wander(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    assert(msg->ranges.size() >= 30);
    int angle;
    int halt = 0;
    int mid = msg->ranges.size() / 2;
    geometry_msgs::Twist cmd;
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
            cmd.angular.z = -1.0;
        } else {
            cmd.angular.z = +1.0;
        }
    } else {
        cmd.linear.x = 1.0;
    }

    outport.write(cmd);
}

ORO_CREATE_COMPONENT(Wander)
