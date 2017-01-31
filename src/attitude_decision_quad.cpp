#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

std_msgs::Float64 roll, pitch, yaw;
geometry_msgs::Vector3 joy;
geometry_msgs::Twist twist;
float linear_x, linear_y, linear_z, angular_z, yaw_begin, yaw_angle, angular_diff;

void callback_roll(const std_msgs::Float64& msg)
{
    roll.data = msg.data;
}

void callback_pitch(const std_msgs::Float64& msg)
{
    pitch.data = msg.data;
}

void callback_yaw(const std_msgs::Float64& msg)
{
    yaw.data = msg.data;
}

void callback_joy(const geometry_msgs::Vector3& msg)
{
    joy.x = msg.x;
    joy.y = msg.y;
    joy.z = msg.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_decision_quad");
    
    ros::NodeHandle node;
    
    ros::Subscriber sub_roll = node.subscribe("roll", 1, callback_roll);
    ros::Subscriber sub_pitch = node.subscribe("pitch", 1, callback_pitch);
    ros::Subscriber sub_yaw = node.subscribe("yaw", 1, callback_yaw);
    ros::Subscriber sub_thrust = node.subscribe("joy", 1, callback_joy);
    
    
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
                
            if(roll.data > 15)
            {
                twist.linear.y = 1;
            }
        
            else if(roll.data < -15)
            {
                twist.linear.y = -1;
            }
        
            else
            {
                twist.linear.y = 0;
            }
            
            
            if(pitch.data > 15)
            {
                twist.linear.x = 1;
            }
        
            else if(pitch.data < -15)
            {
                twist.linear.x = -1;
            }
        
            else
            {
                twist.linear.x = 0;
            }
            
            if(joy.x > 900)
            {
                twist.linear.z = 1;
            }
            
            else if (joy.x < 200)
            {
                twist.linear.z = -1;
            }
            
            else
            {
                twist.linear.z = 0;
            }
            
            yaw_angle = yaw.data;   
                
            angular_diff = yaw_angle - yaw_begin;
            
            if((yaw_begin > 0 && yaw_begin < 270) && (angular_diff > 30 && angular_diff <90))
            {
                ROS_INFO_STREAM("1");
                twist.angular.z = -1;
            }

            else if((yaw_begin > 90 && yaw_begin < 360) && (angular_diff > -90 && angular_diff < -30))
            {
                ROS_INFO_STREAM("2");
                twist.angular.z = 1;
            }
            
            else if((yaw_begin > 270 && yaw_begin < 360) && (yaw_angle > 270 && yaw_angle < 360) && (angular_diff > 30 && angular_diff < 90))
            {
                ROS_INFO_STREAM("3");
                twist.angular.z = -1;
            }

            else if((yaw_begin > 270 && yaw_begin < 360) && (yaw_angle > 0 && yaw_angle < 90) && (angular_diff > -330 && angular_diff < -270))
            {
                ROS_INFO_STREAM("4");
                twist.angular.z = -1;
            }

            else if((yaw_begin > 0 && yaw_begin < 90) && (yaw_angle > 0 && yaw_angle < 90) && (angular_diff > -90 && angular_diff < -30))
            {
                ROS_INFO_STREAM("5");
                twist.angular.z = 1;
            }

            else if((yaw_begin > 0 && yaw_begin < 90) && (yaw_angle > 270 && yaw_angle < 360) && (angular_diff > 270 && angular_diff < 360))
            {
                ROS_INFO_STREAM("6");
                twist.angular.z = 1;
            }

            else twist.angular.z = 0;
            
                        
            
            linear_x = twist.linear.x;
            linear_y = twist.linear.y;
            linear_z = twist.linear.z;
            angular_z = twist.angular.z;
            
            pub_twist.publish(twist);
            
            ROS_INFO_STREAM("Roll: " << roll.data);
            ROS_INFO_STREAM("Pitch: " << pitch.data);
            ROS_INFO_STREAM("Yaw: " << yaw.data);
            ROS_INFO_STREAM("Thrust: " << joy.x);
            
            ROS_INFO_STREAM("Linear X: " << linear_x);
            ROS_INFO_STREAM("Linear Y: " << linear_y);
            ROS_INFO_STREAM("Linear Z: " << linear_z);
            ROS_INFO_STREAM("Angular Z: " << angular_z);
            
        }
        
        ros::spinOnce();
        loop_rate.sleep();
 
    }
    
}














