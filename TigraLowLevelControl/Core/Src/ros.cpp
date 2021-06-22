#include <ros.h>
#include <ros_proto.h>

/**************/
/* ROS things */
/**************/

#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>


// ros::Subscriber<std_msgs::Int8>       topic_speed( "speed_perc", &speed_cb);
// ros::Subscriber<std_msgs::Int8>       topic_steer( "steer_perc", &steer_cb);
// ros::Subscriber<std_msgs::UInt8>      topic_mode( "mode_status", mode_cb );
//=======================================================

/*
 * ROS spin thread - used to receive messages
 */

void rosInit()
{
    /* Serial driver */

    /* ROS setup */
    // ros_node.initNode();
    // ros_node.setSpinTimeout( 20 );

    // /* ROS publishers */

    // /* ROS subscribers */
    // ros_node.subscribe( topic_speed );
    // ros_node.subscribe( topic_steer );
    // ros_node.subscribe( topic_mode );
    /* ROS service client */

    /* Main ROS thread */
}
