#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.0))
        ROS_WARN(" TMOVE : Param close_enough not found, set to 0");
    double dir_threshold;
    if (!nh.param("dir_threshold", dir_threshold, 0.0))
        ROS_WARN(" TMOVE : Param dir_threshold not found, set to 0");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double target_heading = heading(pos_rbt, target);
    double direction = abs(limit_angle(target_heading-ang_rbt))<=M_PI/2 ? 1 : -1;
    double e_ang_prev = direction == 1
        ? limit_angle(target_heading - ang_rbt)
        : limit_angle(target_heading - ang_rbt - M_PI/2);
    double e_lin_prev = dist_euc(pos_rbt, target) * direction;
    double i_lin = 0; double i_ang = 0;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            
            // error
            target_heading = heading(pos_rbt, target);
            direction = abs(M_PI/2-abs(limit_angle(target_heading-ang_rbt))) < dir_threshold 
                ? direction 
                : (abs(limit_angle(target_heading-ang_rbt))<=M_PI/2 ? 1 : -1);
            double e_ang = (direction == 1)
                ? limit_angle(target_heading - ang_rbt)
                : limit_angle(target_heading - ang_rbt - M_PI);
            double e_lin = dist_euc(pos_rbt, target) * direction;

            // PID
            double p_lin = Kp_lin * e_lin;
            i_lin += Ki_lin * e_lin * dt;
            double d_lin = Kd_lin * (e_lin - e_lin_prev) / dt;
            double p_ang = Kp_ang * e_ang;
            i_ang += Ki_ang * e_ang * dt;
            double d_ang = Kd_ang * (e_ang - e_ang_prev) / dt;

            // set speed
            double unsat_lin_vel = p_lin + i_lin + d_lin;
            double sat_lin_acc = saturate(max_lin_acc, -max_lin_acc, (unsat_lin_vel-cmd_lin_vel)/dt);
            cmd_lin_vel = saturate(max_lin_vel, -max_lin_vel, cmd_lin_vel+sat_lin_acc*dt)* (1 - abs(e_ang/(M_PI/2)));
            double unsat_ang_vel = p_ang + i_ang + d_ang;
            double sat_ang_acc = saturate(max_ang_acc, -max_ang_acc, (unsat_ang_vel-cmd_ang_vel)/dt);
            cmd_ang_vel = saturate(max_ang_vel, -max_ang_vel, cmd_ang_vel+sat_ang_acc*dt);
            e_lin_prev = e_lin;
            e_ang_prev = e_ang;

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
                ROS_INFO(" TMOVE : Target(%6.3f, %6.3f)  Current(%6.3f, %6.3f)", target.x, target.y, 
                        pos_rbt.x, pos_rbt.y);
                ROS_INFO(" TMOVE : E_LIN: %6.3f E_ANG: %6.3f", e_lin, e_ang);
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}
