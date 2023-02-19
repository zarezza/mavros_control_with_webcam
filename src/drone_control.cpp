#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

mavros_msgs::PositionTarget global_vel_cmd;
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);

bool near_equal(double a, double b, double precision) {
    return std::fabs(a - b) < precision;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist> 
            ("velocity_commands", 10, velocityCallback);
    ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    mavros_msgs::PositionTarget vel_cmd;
    vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                        mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    ros::Rate rate(30);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //arming
    bool armed = false;
    while(ros::ok() && !armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed = true;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    bool tookOff = false;
    ros::Time takingoff;

    while(ros::ok() && !tookOff) {
        ros::spinOnce();
        rate.sleep();

        if(!near_equal(current_pose.pose.position.z, 5, 0.1)) {
            pose.pose.position.x = current_pose.pose.position.x;
            pose.pose.position.y = current_pose.pose.position.y;
            pose.pose.position.z = 5.0;

            local_pos_pub.publish(pose);
        }

        if(takingoff.is_zero()) {
            takingoff = ros::Time::now();
            continue;
        }

        if(ros::Time::now() - takingoff < ros::Duration(5.0)) {
            continue;
        }

        tookOff = true;
    }

    while(ros::ok()) {

        ros::spinOnce();
        vel_pub.publish(global_vel_cmd);
        rate.sleep();
    }
    return 0;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    mavros_msgs::PositionTarget vel_cmd;
    vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ;

    vel_cmd.velocity.x  = msg->linear.x;
    vel_cmd.velocity.y  = msg->linear.y;    
    vel_cmd.velocity.z  = msg->linear.z;
    vel_cmd.yaw_rate    = msg->angular.z;


    global_vel_cmd = vel_cmd;
    ROS_INFO("Received: vel[%f, %f, %f], ang[%f]", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
}
    