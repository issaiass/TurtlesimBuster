#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
using namespace ros;
using namespace std_msgs;

int main(int argc, char **argv) {
    init(argc, argv, "talker");
    NodeHandle n;
    Publisher chatter_pub = n.advertise<String>("chatter", 1000);
    Rate loop_rate(10);

    while(ok()) {
        String msg;

        string message = "hello visual studio";
        msg.data = message;
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}