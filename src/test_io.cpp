#include "ur_modern_driver/ur_driver.h"
#include "ur_modern_driver/ur_hardware_interface.h"
#include "ur_modern_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>
#include <cstdlib>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_ur_io");
//    if (argc != 3)
//    {
//        ROS_INFO("usage: add_two_ints_client X Y");
//        return 1;
//    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
    ur_msgs::SetIO srv;
    srv.request.fun = 1; 
    srv.request.pin = 2;
    srv.request.state = atof(argv[1]);
    if (client.call(srv))
    {
        ROS_INFO("Succeed!");
    }
    else
    {
        ROS_ERROR("Failed to call service setIO");
        return 1;
    }

    return 0;
}
