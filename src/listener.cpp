#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
using namespace ros;
using namespace std_msgs;

void chatterCallback(const String::ConstPtr &msg);

int main(int argc, char **argv) {
    init(argc, argv, "listener");
    NodeHandle n;
    
    Subscriber sub = n.subscribe("chatter", 100, chatterCallback);

    /* spin will block the code */
    // spin() 
    /* or below */
    while(ok()) {
        spinOnce();
    }

    return 0;
}

void chatterCallback(const String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}