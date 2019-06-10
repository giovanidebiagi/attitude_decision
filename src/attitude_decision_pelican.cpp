#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <asctec_hl_comm/mav_ctrl.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Bool.h"

std_msgs::Float64 roll, pitch, heading, initial_heading;
geometry_msgs::Vector3 joy;

/*! Code for Pelican quadrotor */
asctec_hl_comm::mav_ctrl vel_ctrl;

/*! This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */
std_msgs::Bool reset;

/*! This is used to prevent conversion node from reseting if attitude_decision is in joystik mode */
std_msgs::Bool operation_mode;

static ros::Publisher mav_pub;  /*! Diego's code */
bool readyToControl = false;    /*! Diego's code. If this is false, the code doesnt publish anything. */

bool lastWill = false;

float linear_x, linear_y, linear_z, angular_z;

/*! Boolean variable to choose from controlling yaw rate using either gyroscope or joystick.
    
    joystick -> operation_mode = false
    gyro -> operation_mode = true */
// bool operation_mode = false;

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

/*! Diego's code for Pelican */
void callback_rc(const sensor_msgs::Joy::ConstPtr& msg)
{

//   if (mav_pub) {
    asctec_hl_comm::mav_ctrl mav;   
    mav.header.stamp = msg->header.stamp;

    // Begin Message Realocation
    if (msg->axes[6] > 0)
    {
      readyToControl = true;    /*! Makes the code able to publish the message */
    }

    else 
    {
        readyToControl = false;
    }
//   }
}

void callback_reset(const std_msgs::Bool& msg)
{
    reset.data = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_decision_pelican");
    
    ros::NodeHandle node;
    
    ros::Subscriber sub_roll = node.subscribe("roll", 1, callback_roll);
    ros::Subscriber sub_pitch = node.subscribe("pitch", 1, callback_pitch);
    ros::Subscriber sub_heading = node.subscribe("heading", 1, callback_heading);
    ros::Subscriber sub_thrust = node.subscribe("joy", 1, callback_joy);
    ros::Subscriber sub_reset = node.subscribe("reset", 1, callback_reset); /*!< This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */

    /*! Code for Pelican quadrotor */
    ros::Subscriber sub = node.subscribe("fcu/rc", 1, callback_rc); /*! Subscribed to the radio controller to check if its sending commands */

    /*! Code for Pelican quadrotor */
    ros::Publisher pub_operation_mode = node.advertise<std_msgs::Bool>("/operation_mode",1);
    ros::Publisher pub_vel = node.advertise<asctec_hl_comm::mav_ctrl>("/fcu/control",1);

    ros::Rate loop_rate(50);

    operation_mode.data = false;
    joy.z = 1;
    reset.data = false; /*!< This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */

    /*!  Using launch file, it is necessary that this node waits for the imu_node to open
         the serial port and start publishing data, otherwise the conversion_node will have
         garbage values and attitude_decision nodes will abort operation */
    usleep(5000000);  

    while(ros::ok())
    {        
        flag = false;

        /*! Code for Pelican quadrotor */
        vel_ctrl.type = 4;  /*! Control mode of body velocities */
        vel_ctrl.x = 0;
        vel_ctrl.y = 0;
        vel_ctrl.z = 0;
        vel_ctrl.yaw = 0;

        if(joy.z == 0 && previous_joy_state == 0)
        {
            if(operation_mode.data == false) operation_mode.data = true;
            else if(operation_mode.data == true) operation_mode.data = false;
            
            previous_joy_state = 1;
        }

        else if(joy.z == 1)
        {
            previous_joy_state = 0;
        }

        if(roll.data > 15)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.y = -0.3;

            flag = true;
        }
        
        else if(roll.data < -15)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.y = 0.3;

            flag = true;
        }
        
        else 
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.y = 0;
        }
            
        if(joy.x > 900)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.z = -0.3;

            flag = true;
        }
            
        else if (joy.x < 200)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.z = 0.3;

            flag = true;                
        }

        else
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.z = 0;  
        }

        if(operation_mode.data == false)
        {    
            if(joy.y > 900)
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = -0.3;
            }

            else if(joy.y < 200)
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = 0.3;
            }

            else
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = 0;
            }

            /*! Auxiliary variable used to, when gyro mode is activated, it starts with
            heading = 0 */
            initial_heading.data = heading.data;
        }

        else if(operation_mode.data == true)
        {
            /*! This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */
            if(reset.data == true)
            {
                initial_heading.data = 0;
                reset.data = false;
            }

            /*! When gyro mode is activated, it starts with heading = 0 */
            heading.data = heading.data - initial_heading.data;

            if(heading.data > 25)
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = -0.3;
            }

            else if(heading.data < -25)
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = 0.3;
            }

            else
            {
                /*! Code for Pelican quadrotor */
                vel_ctrl.yaw = 0;
            }
        }

        if(pitch.data > 15)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.x = -0.3;

            flag = true;
        }
        
        else if(pitch.data < -15 && pitch.data > -45)
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.x = 0.3;

            flag = true;               
        }

        else if(pitch.data < -45)
        {           
            /*! Code for Pelican quadrotor */
            vel_ctrl.x = 0;
            vel_ctrl.y = 0;
            vel_ctrl.z = 0;
            vel_ctrl.yaw = 0;

            linear_x = vel_ctrl.x;
            linear_y = vel_ctrl.y;
            linear_z = vel_ctrl.z;
            angular_z = vel_ctrl.yaw;

            ROS_INFO_STREAM("Linear X: " << linear_x);
            ROS_INFO_STREAM("Linear Y: " << linear_y);
            ROS_INFO_STREAM("Linear Z: " << linear_z);
            ROS_INFO_STREAM("Angular Z: " << angular_z);
            
            /*! Publish message for Pelican quadrotor */
            pub_vel.publish(vel_ctrl);

            ROS_INFO_STREAM("Operation aborted. Please, restart the system.");
            
            break;
        }
                
        else
        {
            /*! Code for Pelican quadrotor */
            vel_ctrl.x = 0;
        } 

        /*! Code for Pelican quadrotor */
        linear_x = vel_ctrl.x;
        linear_y = vel_ctrl.y;
        linear_z = vel_ctrl.z;
        angular_z = vel_ctrl.yaw;
    
        if(operation_mode.data == false)
        {
            ROS_INFO_STREAM("Heading control: Joystick. Press the joystick button to switch.");
        }

        else if(operation_mode.data == true)
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

        pub_operation_mode.publish(operation_mode);

        /*! If it is desired that this node only publishes to the topic when it has velocities
        commands different than 0, please uncomment this conditional structure.
        If you want to have another controller alongside with this node, and switch between them,
        for example, the conditional structure MUST be uncommented, in order to let the topic
        free when this controller node is not being used (velocities desired here are = 0) . */
        // if(flag == true)
        // {
                /*! Code for Pelican quadrotor  */
                if(readyToControl == true)
                {
                    pub_vel.publish(vel_ctrl);
                    
                    /*! Auxiliary variable used to make velocities 0 before publishing for the
                    last time and giving control to the radio controller */
                    lastWill = true;
                }

                else if(readyToControl == false)
                {
                    /*! Turns all commands into 0 before publishing for the last time and giving
                    control to the radio controller */
                    vel_ctrl.x = 0;
                    vel_ctrl.y = 0;
                    vel_ctrl.z = 0;
                    vel_ctrl.yaw = 0;

                    if(lastWill == true)
                    {
                        pub_vel.publish(vel_ctrl);
                        lastWill = false;
                    }
                }     
        //  }  

        ros::spinOnce();
        loop_rate.sleep();
    }
}













