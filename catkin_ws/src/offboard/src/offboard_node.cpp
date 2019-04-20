#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::Twist vel;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void receive_callback(const geometry_msgs::Twist::ConstPtr& msg){
     vel = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber cmd_receive = nh.subscribe<geometry_msgs::Twist>
      ("cmd_vel/", 10, receive_callback);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //geometry_msgs::Twist vel;
    // vel.linear.x = 0;
    //pose.pose.position.y = 0;
    //pose.pose.position.z = 0;

    //send a few setpoints before starting
    //    for(int i = 100; ros::ok() && i > 0; --i){
    //    local_pos_pub.publish(pose);
    //    ros::spinOnce();
    //    rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //while(ros::ok() &&(current_state.mode != "OFFBOARD") ||(!current_state.arme) ){
    while(ros::ok()){
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
                }
                last_request = ros::Time::now();
            }
        }

	local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }
    //ros::spin();

    return 0;
}
