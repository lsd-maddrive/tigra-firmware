#include <ros.h>
#include <ros_proto.h>

/**************/
/* ROS things */
/**************/

#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tigra_msgs/TigraState.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define RAD_S_TO_RPM 9.55

osThreadId ROSSpinThreadHandle;
ros::NodeHandle ros_node;
uint16_t reciveWachdog=0;

void ROSReciveFeedback(const tigra_msgs::TigraState &msg)
{
    // char str[50];
    // sprintf(str,"ROS Speed:%d Angle:%d\n\r",(int)msg.rotation_speed,(int)msg.angle_steering);
    // printDebugMessage((uint8_t*)str);
    setReferenceSpeed((float)msg.rotation_speed*RAD_S_TO_RPM);
    reciveWachdog=0;
}

ros::Subscriber<tigra_msgs::TigraState> topicInSpeed("state_cmd", &ROSReciveFeedback);
tigra_msgs::TigraState outMsg;
ros::Publisher topicOutSpeed("state", &outMsg);


/*
 * ROS spin thread - used to receive messages
 */
void ROSSpinThreadTask(void const * argument)
{
    uint16_t timer=0;
    while(true)
    {
        if(timer==5)
        {
            timer=0;
            ///outSpeed.data=(int)getSpeed();
            outMsg.angle_steering=0;
            outMsg.rotation_speed=getSpeed()/RAD_S_TO_RPM;
            outMsg.stamp.sec=HAL_GetTick()/1000;
            outMsg.stamp.nsec=(HAL_GetTick()%1000)*1000000;
            topicOutSpeed.publish(&outMsg);
        }
        else
            timer++;
        if(reciveWachdog==100)
        {
            setReferenceSpeed(0);
            rosInit();
            printDebugMessage((uint8_t*)"TCP connection close\n\r");
            reciveWachdog=0;
        }
        else
        {
            reciveWachdog++;
        }
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
    osThreadDef(ROSSpinThread, ROSSpinThreadTask, osPriorityAboveNormal, 0, 2048);
    ROSSpinThreadHandle = osThreadCreate(osThread(ROSSpinThread), NULL);

    /* Main ROS thread */
}
