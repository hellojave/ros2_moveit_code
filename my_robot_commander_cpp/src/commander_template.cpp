#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>
#include <example_interfaces/msg/bool.hpp>

using MoveGroupInterface =moveit::planning_interface::MoveGroupInterface;
using Bool=example_interfaces::msg::Bool;
using namespace std::placeholders;


class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_=std::make_shared<MoveGroupInterface>(node_,"arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        arm_->setPlanningTime(5.0);                // 加规划时间，更稳定
        arm_->setNumPlanningAttempts(3);           // 多次尝试
        gripper_=std::make_shared<MoveGroupInterface>(node_,"gripper");

        
        open_gripper_sub_ =node_->create_subscription<Bool>(
            "open_gripper",10,std::bind(&Commander::OpenFripperCallback,this,_1));


    }
    void goToNameTarget(const std::string &name)
    {

        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExcute(arm_);
    }

    void goToJointTarget(const std::vector<double> joint)
    {

        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint);
        planAndExcute(arm_);
    }

    void goToPoseTarget(double x,double y,double z,double roll,double pitch,double yaw,bool cartesion_path=false)
    {
        tf2::Quaternion q;

        q.setRPY(3.14,0,0);
        q=q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id="base_link";

        target_pose.pose.position.x=x;
        target_pose.pose.position.y=y;
        target_pose.pose.position.z=z;

        target_pose.pose.orientation.x=q.getX();
        target_pose.pose.orientation.y=q.getY();
        target_pose.pose.orientation.z=q.getZ();
        target_pose.pose.orientation.w=q.getW();


        arm_->setStartStateToCurrentState();
        if(cartesion_path==false)
        {
            arm_->setPoseTarget(target_pose);
            planAndExcute(arm_);

        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoint;
            waypoint.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction=arm_->computeCartesianPath(waypoint,0.01,0.0,trajectory,true, nullptr);
            if(fraction==1)
            {
                arm_->execute(trajectory);
            }
        }
    }
    void gripper_open()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        planAndExcute(gripper_);
    }
    void gripper_close()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");
        planAndExcute(gripper_);

    }

private:
    void planAndExcute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success=(interface->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success)
        {
            interface->execute(plan);
        }

    } 
    
    void OpenFripperCallback(const Bool &msg)
    {
        if(msg.data)
        {
            gripper_open();
        }
        else{
            gripper_close();
        }
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("commander");
    auto commander =Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

