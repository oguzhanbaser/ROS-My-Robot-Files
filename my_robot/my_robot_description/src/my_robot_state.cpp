#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_robot_state");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);

    const double degree = M_PI / 180;
    
    double tilt = 0, tinc = degree, swivel = 0, angle = 0, height = 0, hinc = 0.005;

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while(ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
        joint_state.name[0] = "front_wheel_joint";
        joint_state.position[0] = tilt;
        joint_state.name[1] = "left_motor_joint";
        joint_state.position[1] = 0;
        joint_state.name[2] = "right_motor_joint";
        joint_state.position[2] = 0;
        joint_state.name[3] = "left_wheel_joint";
        joint_state.position[3] = tilt;
        joint_state.name[4] = "right_wheel_joint";
        joint_state.position[5] = tilt;


        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = tilt;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle + M_PI / 2);

        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        tilt += tinc;
        if(tilt < -0.5 || tilt > 0) tinc *= -1;
        
        height += hinc;
        if(height > 0.2 || height < 0) hinc *= -1;

        swivel += degree;
        //angle += degree / 4;

        loop_rate.sleep();
    }

    return 0;
}
