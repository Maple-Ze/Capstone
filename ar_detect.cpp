#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sound_play.h"

int id_num = 0;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    sound_play::SoundClient sc;
    sleep(2);
    if (msg->markers.size() > 0)
    {
        ROS_INFO("[%d]", msg->markers[0].id);
        id_num = msg->markers[0].id;
        switch (msg->markers[0].id)
        {
        case 0:
            sc.say("Zero");
            sleep(2);
            // ros::shutdown();
            break;
        case 1:
            sc.say("One");
            sleep(2);
            break;
        case 2:
            sc.say("Two");
            sleep(2);
            break;
        case 3:
            sc.say("Three");
            sleep(2);
            break;
        case 4:
            sc.say("Four");
            sleep(2);
            break;
        case 5:
            sc.say("Five");
            sleep(2);
            break;
        case 6:
            sc.say("Six");
            sleep(2);
            break;
        case 7:
            sc.say("Seven");
            sleep(2);
            break;
        case 8:
            sc.say("Eight");
            sleep(2);
            break;
        case 9:
            sc.say("Nine");
            sleep(2);
            break;
        case 10:
            sc.say("Ten");
            sleep(2);
            break;
        default:
            break;
        }
    }
    else
    {
        ROS_INFO("Cannot find AR tag");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_detect");

    ros::NodeHandle n;
    // ROS_INFO("[%d]", 1);
    // ros::Rate loop
    ros::Subscriber sub = n.subscribe("/ar_pose_marker", 10, chatterCallback);
    // ROS_INFO("[%d]", 2);

    ros::spin();
    // ROS_INFO("[%d]", 5);
    return 0;
}