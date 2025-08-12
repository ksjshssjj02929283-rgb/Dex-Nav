#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <cmath>

geometry_msgs::PoseStamped odom_pose_;
geometry_msgs::PoseStamped goal_pose_;
bool get_goal_flag_ = false;

// for smooth controller
double k1_ = 1.0;
double k2_ = 4.0;

void callbackOdom(const nav_msgs::Odometry::ConstPtr &odom)
{
    odom_pose_.header = odom->header;
    odom_pose_.pose = odom->pose.pose;
}

void callbackGoal(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    get_goal_flag_ = true;
    goal_pose_ = *goal;
}

// 函数用于将角度限制在 -pi 到 pi 范围内
double NormalizeAngle(const double angle)
{
    double a = std::fmod(angle, 2.0 * M_PI);
    if (a < -M_PI)
    {
        a += (2.0 * M_PI);
    }
    else if (a >= M_PI)
    { // 这里一般不加等号，为了和Apollo代码保持完全一致（映射到[-pi,pi))才加了等号。
        a -= (2.0 * M_PI);
    }
    return a;
}

double getTheta(const geometry_msgs::Pose &odom_pose, const geometry_msgs::Pose &goal_pose)
{
    double head_yaw = std::atan2(goal_pose.position.y - odom_pose.position.y, goal_pose.position.x - odom_pose.position.x);
    double goal_yaw = tf::getYaw(goal_pose.orientation);
    double theta = goal_yaw - head_yaw;
    return NormalizeAngle(theta);
}

double getDelta(const geometry_msgs::Pose &odom_pose, const geometry_msgs::Pose &goal_pose)
{
    double head_yaw = std::atan2(goal_pose.position.y - odom_pose.position.y, goal_pose.position.x - odom_pose.position.x);
    double odom_yaw = tf::getYaw(odom_pose.orientation);
    double delta = odom_yaw - head_yaw;
    return NormalizeAngle(delta);
}

double getDistance(const geometry_msgs::Pose &odom_pose, const geometry_msgs::Pose &goal_pose)
{
    return std::sqrt(std::pow(odom_pose.position.x - goal_pose.position.x, 2) + std::pow(odom_pose.position.y - goal_pose.position.y, 2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smooth_control_node");
    ros::NodeHandle nh("");

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("zgi_cmd_vel", 10);

    ros::Subscriber odom_sub = nh.subscribe("odom", 10, callbackOdom);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, callbackGoal);

    ros::Rate rate(20);
    while (ros::ok())
    {

        if (get_goal_flag_)
        {
            double theta = getTheta(odom_pose_.pose, goal_pose_.pose);
            double delta = getDelta(odom_pose_.pose, goal_pose_.pose);
            double dist = getDistance(odom_pose_.pose, goal_pose_.pose);

            if (dist <= 0.1)
            {
                geometry_msgs::Twist twist;
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                get_goal_flag_ = false;
            }
            else
            {
                // 这种比较长的公式就是容易抄错，要小心检查 error formula !!!
                // double k = -1.0 / dist * ((k2_ * (delta - std::atan(-k1_ * theta))) + (1.0 + k1_ / (1.0 + std::pow(k1_ * theta, 2) * std::sin(delta))));
                // 这种比较长的公式就是容易抄错，要小心检查 right formula !!!
                double k = -1.0 / dist * ((k2_ * (delta - std::atan(-k1_ * theta))) + (1.0 + k1_ / (1.0 + std::pow(k1_ * theta, 2))) * std::sin(delta));
                double v = 0.8;
                double w = v * k;
                geometry_msgs::Twist twist;
                twist.linear.x = v;
                twist.angular.z = w;
                twist_pub.publish(twist);
            }
        }
        else
        {
            double v = 0.0;
            double w = 0.0;
            geometry_msgs::TwistStamped ps;
            ps.header.frame_id = "odom";
            ps.twist.linear.x = v;
            ps.twist.angular.z = w;
            twist_pub.publish(ps);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}