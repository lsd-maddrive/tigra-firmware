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
#define RAD_TO_DEG 57.3 

osThreadId ROSSpinThreadHandle;
ros::NodeHandle ros_node;
uint16_t reciveWachdog=0;
extern uint8_t reconnectFlag;

class AlphaFilter {
public:
    AlphaFilter(float alpha) {
        m_alpha = alpha;
        m_memory = 0;
    }

    float getFiltered(float value) {
        float new_value = m_memory * (1-m_alpha) + value * m_alpha;
        m_memory = new_value;
        return new_value;
    }

private:
    float m_alpha;
    float m_memory;
};

AlphaFilter steer_filter(1);

void ROSReciveFeedback(const tigra_msgs::TigraState &msg)
{
    char str[50];

    //float speed=getSpeed();
    float speed=getFilteredSpeed();
    float angle=msg.angle_steering*RAD_TO_DEG; 
    setReferenceSpeed((float)msg.rotation_speed*RAD_S_TO_RPM);
    //if(speed>=10 || speed<=-10)
    //{
    if(angle>20) angle=20;
    if(angle<-20) angle=-20;

    float filtered = steer_filter.getFiltered(angle);

    // To avoid rotation disable
    /*if (((int)filtered) == 0) {
        filtered = -1;
    }*/

    sendReferenceAngle(filtered);
    sprintf(str,"ROS Speed:%2.2f Angle:%d\n\r",msg.rotation_speed,(int)filtered);     
    //printDebugMessage((uint8_t*)str);
    //}
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
            outMsg.angle_steering=(float)(-1*(getAngle()/RAD_TO_DEG));
            outMsg.rotation_speed=getSpeed()/RAD_S_TO_RPM;
            outMsg.stamp.sec=HAL_GetTick()/1000;
            outMsg.stamp.nsec=(HAL_GetTick()%1000)*1000000;
            topicOutSpeed.publish(&outMsg);
        }
        else
            timer++;
        if(reciveWachdog==100)
        {
            //setReferenceSpeed(0);
            //sendReferenceAngle(0); // Turn off rotation
            //rosInit();
            //printDebugMessage((uint8_t*)"TCP connection close\n\r");
            reciveWachdog=0;
        }
        else
        {
            reciveWachdog++;
        }
        if(reconnectFlag==1)
        {
           rosInit(); 
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
