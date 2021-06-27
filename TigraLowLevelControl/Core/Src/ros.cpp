#include <ros.h>
#include <ros_proto.h>

/**************/
/* ROS things */
/**************/

#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


osThreadId ROSSpinThreadHandle;
ros::NodeHandle ros_node;


// ros::Subscriber<std_msgs::Int8>       topic_speed( "speed_perc", &speed_cb);
// ros::Subscriber<std_msgs::Int8>       topic_steer( "steer_perc", &steer_cb);
// ros::Subscriber<std_msgs::UInt8>      topic_mode( "mode_status", mode_cb );
//=======================================================

void ROSspeedReciveFeedback(const std_msgs::Int8 &msg)
{
    char str[50];
    if((int)msg.data<0)
    {
        sprintf(str,"ROS Speed:-%d\n\r",(int)msg.data);
    }
    else
        sprintf(str,"ROS Speed:%d\n\r",(int)msg.data);
    printDebugMessage((uint8_t*)str);
}

ros::Subscriber<std_msgs::Int8> topicInSpeed("InSpeed", &ROSspeedReciveFeedback);
std_msgs::Int8 outSpeed;
ros::Publisher topicOutSpeed("outSpeed", &outSpeed);

/*
 * ROS spin thread - used to receive messages
 */
void ROSSpinThreadTask(void const * argument)
{
    uint16_t timer=0;
    while(true)
    {
        if(timer==50)
        {
            timer=0;
            outSpeed.data=(int)getSpeed();
            topicOutSpeed.publish(&outSpeed);
        }
        else
            timer++;
        ros_node.spinOnce();
        osDelay(10);
    }
}

void rosInit()
{    
    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout(20);
    /* ROS publishers */
    ros_node.advertise(topicOutSpeed);
    /* ROS subscribers */
    ros_node.subscribe(topicInSpeed);
    /* ROS service client */
    osThreadDef(ROSSpinThread, ROSSpinThreadTask, osPriorityAboveNormal, 0, 1024);
    ROSSpinThreadHandle = osThreadCreate(osThread(ROSSpinThread), NULL);
    /* Main ROS thread */
}
