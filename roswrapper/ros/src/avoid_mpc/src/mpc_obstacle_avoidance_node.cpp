#include "AvoidanceStateMachine.h"
#include "ParameterManager.h"
#include <ros/ros.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "avoid_mpc");
    ros::NodeHandle nodeHandle("~");
    Param::GetInstance().init(nodeHandle);
    AvoidanceStateMachine node(nodeHandle);
    ros::spin();
    return 0;
}
