#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

std_msgs::Float64 pitch, yaw;

float linear, angular, yaw_angle, yaw_begin, angular_diff;


void callback_pitch(const std_msgs::Float64& msg)
{
    pitch.data = msg.data;     /*!< According to the hand axis */ 
}
    
void callback_yaw(const std_msgs::Float64& msg)
{
    yaw.data = msg.data;
}

int main(int argc, char **argv)
{    
    ros::init (argc, argv, "attitude_node");
    ros::NodeHandle node;
    
    geometry_msgs::Twist twist;
                
    ros::Subscriber sub_pitch = node.subscribe("pitch", 1, callback_pitch);
    ros::Subscriber sub_yaw = node.subscribe("yaw", 1, callback_yaw);
    
    ros::Publisher pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    
    ros::Rate loop_rate(50);

    int i = 0; 
           
    while(ros::ok())
    {       
        /*! Calibration */
        if(i<=200)
        {   
            if (i==0) ROS_INFO_STREAM("Calibrating sensors...");
            yaw_begin = yaw.data;
            twist.linear.x = 0;
            i++;
        }
        
        else if (i>200)
        {
            yaw_angle = yaw.data;   
                
            angular_diff = yaw_angle - yaw_begin;
        
            if(pitch.data > 15)
            {
                twist.linear.x = 0.6;
            }
        
            else if(pitch.data < -15)
            {
                twist.linear.x = -0.6;
            }
        
            else
            {
                twist.linear.x = 0;
            }
            
        
            if((yaw_begin > 0 && yaw_begin < 270) && (angular_diff > 30 && angular_diff <90))
            {
                ROS_INFO_STREAM("1");
                twist.angular.z = 0.4;
            }

            else if((yaw_begin > 90 && yaw_begin < 360) && (angular_diff > -90 && angular_diff < -30))
            {
                ROS_INFO_STREAM("2");
                twist.angular.z = -0.4;
            }
            
            else if((yaw_begin > 270 && yaw_begin < 360) && (yaw_angle > 270 && yaw_angle < 360) && (angular_diff > 30 && angular_diff < 90))
            {
                ROS_INFO_STREAM("3");
                twist.angular.z = 0.4;
            }

            else if((yaw_begin > 270 && yaw_begin < 360) && (yaw_angle > 0 && yaw_angle < 90) && (angular_diff > -330 && angular_diff < -270))
            {
                ROS_INFO_STREAM("4");
                twist.angular.z = 0.4;
            }

            else if((yaw_begin > 0 && yaw_begin < 90) && (yaw_angle > 0 && yaw_angle < 90) && (angular_diff > -90 && angular_diff < -30))
            {
                ROS_INFO_STREAM("5");
                twist.angular.z = -0.4;
            }

            else if((yaw_begin > 0 && yaw_begin < 90) && (yaw_angle > 270 && yaw_angle < 360) && (angular_diff > 270 && angular_diff < 360))
            {
                ROS_INFO_STREAM("6");
                twist.angular.z = -0.4;
            }

            else twist.angular.z = 0;

            linear = twist.linear.x;
            angular = twist.angular.z;

            pub_twist.publish(twist);

            ROS_INFO_STREAM("Linear: " << linear);
            ROS_INFO_STREAM("Angular: " << angular);
            ROS_INFO_STREAM("yaw.data: " << yaw_angle);
            ROS_INFO_STREAM("yaw_begin: " << yaw_begin);
            ROS_INFO_STREAM("angular_diff: " << angular_diff); 
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
 }