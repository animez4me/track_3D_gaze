//save compiler switches
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
//restore compiler switches
#pragma GCC diagnostic pop

#include "track.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "track_3D_gaze_node");

    ros::NodeHandle n;
    ros::Publisher fixation_pub = n.advertise<geometry_msgs::Pose>("fixation_3d", 1000);

    //track_3d_fixation();
    geometry_msgs::Pose fixation;


//    fixation.position.x= fixed_pt.x;
//    fixation.position.y= fixed_pt.y;
//    fixation.position.z= fixed_pt.z;
    ROS_INFO_STREAM("3D fixation: " << fixation);

    //fixation_pub.publish(fixation);

    // @todo: Mike, here you should publish to ROS the following:
    // "currentPOR.is_fixation" : this is going to trigger ORK (fixation classified according to duel time)
    // "et3d.current_3Dfixation" : is the current 3D fixation you need, but you'll use it if currentPOR.is_fixation == true


  return 0;
}
