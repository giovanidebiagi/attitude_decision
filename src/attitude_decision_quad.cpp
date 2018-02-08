#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

std_msgs::Float64 roll, pitch, heading;
geometry_msgs::Vector3 joy;
geometry_msgs::Twist twist;
float linear_x, linear_y, linear_z, angular_z;

/*! Boolean variable to choose from controlling yaw rate using either gyroscope or joystick.
    
    joystick -> operation_mode = false
    gyro -> operation_mode = true */
bool operation_mode = false;

/*! Boolean variable used to only send commands if the user is willing to.
    This keeps the command topics free unless there are commands to be sent, therefore, it allows
    other controller to actuate while there are no commands being sent by the glove 
    
    sending commands -> flag = true
    not sending commands -> flag = false */
bool flag = false;

/*! Boolean variable to abort operation in case of emergency */
bool aborting_flag = false;

int previous_joy_state = 0;

void callback_roll(const std_msgs::Float64& msg)
{
    roll.data = msg.data;
}

void callback_pitch(const std_msgs::Float64& msg)
{
    pitch.data = msg.data;
}

void callback_heading(const std_msgs::Float64& msg)
{
    heading.data = msg.data;
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
    ros::Subscriber sub_heading = node.subscribe("heading", 1, callback_heading);
    ros::Subscriber sub_thrust = node.subscribe("joy", 1, callback_joy);
    
    ros::Publisher pub_twist = node.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
    
    ros::Rate loop_rate(50);

    joy.z = 1;
    
    while(ros::ok())
    {        
        flag = false;
            
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.z = 0;

        if(joy.z == 0 && previous_joy_state == 0)
        {
            if(operation_mode == false) operation_mode = true;
            else if(operation_mode == true) operation_mode = false;
            
            previous_joy_state = 1;
        }

        else if(joy.z == 1)
        {
            previous_joy_state = 0;
        }

        if(roll.data > 15)
        {
            twist.linear.y = -0.3;
            flag = true;
        }
        
        else if(roll.data < -15)
        {
            twist.linear.y = 0.3;
            flag = true;
        }
        
        else 
        {
            twist.linear.y = 0;
        }
            
        if(joy.x > 900)
        {
            twist.linear.z = -0.3;
            flag = true;
        }
            
        else if (joy.x < 200)
        {
            twist.linear.z = 0.3;
            flag = true;                
        }

        else
        {
            twist.linear.z = 0;
        }

        if(operation_mode == false)
        {    
            if(joy.y > 900)
            {
                twist.angular.z = 0.3;
            }

            else if(joy.y < 200)
            {
                twist.angular.z = -0.3;
            }

            else
            {
                twist.angular.z = 0;
            }
        }

        else if(operation_mode == true)
        {
            if(heading.data > 25)
            {
                twist.angular.z = 0.3;
            }

            else if(heading.data < -25)
            {
                twist.angular.z = -0.3;
            }

            else
            {
                twist.angular.z = 0;
            }
        }

        if(pitch.data > 15)
        {
            twist.linear.x = 0.3;
            flag = true;
        }
        
        else if(pitch.data < -15 && pitch.data > -45)
        {
            twist.linear.x = -0.3;
            flag = true;               
        }

        else if(pitch.data < -45)
        {
            /*! Set all velocities to 0 */
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.z = 0;

            linear_x = twist.linear.x;
            linear_y = twist.linear.y;
            linear_z = twist.linear.z;
            angular_z = twist.angular.z;

            ROS_INFO_STREAM("Linear X: " << linear_x);
            ROS_INFO_STREAM("Linear Y: " << linear_y);
            ROS_INFO_STREAM("Linear Z: " << linear_z);
            ROS_INFO_STREAM("Angular Z: " << angular_z);
            
            pub_twist.publish(twist);

            ROS_INFO_STREAM("Operation aborted. Please, restart the system.");
            
            break;
        }
                
        else
        {
            twist.linear.x = 0;
        } 

        linear_x = twist.linear.x;
        linear_y = twist.linear.y;
        linear_z = twist.linear.z;
        angular_z = twist.angular.z;
 

        if(operation_mode == false)
        {
            ROS_INFO_STREAM("Heading control: Joystick. Press the joystick button to switch.");
        }

        else if(operation_mode == true)
        {
            ROS_INFO_STREAM("Heading control: Gyroscope. Press the joystick button to switch.");
        }    

        ROS_INFO_STREAM("Roll: " << roll.data);
        ROS_INFO_STREAM("Pitch: " << pitch.data);
        ROS_INFO_STREAM("Heading (gyro): " << heading.data);
        ROS_INFO_STREAM("Heading (joystick): " << joy.y);
        ROS_INFO_STREAM("Thrust: " << joy.x);
                
        ROS_INFO_STREAM("Linear X: " << linear_x);
        ROS_INFO_STREAM("Linear Y: " << linear_y);
        ROS_INFO_STREAM("Linear Z: " << linear_z);
        ROS_INFO_STREAM("Angular Z: " << angular_z);


        /*! If it is desired that this node only publishes to the topic when it has velocities
        commands different than 0, please uncomment this conditional structure.
        If you want to have another controller alongside with this node, and switch between them,
        for example, the conditional structure MUST be uncommented, in order to let the topic
        free when this controller node is not being used (velocities desired here are = 0) . */
        //if(flag == true)
        //{
            pub_twist.publish(twist);
        //  }  

        ros::spinOnce();
        loop_rate.sleep();
    }
}














