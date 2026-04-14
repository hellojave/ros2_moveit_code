#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_moveit");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 后台 spinning 线程
  auto spinner = std::thread([&executor]() {
    executor.spin();
  });

  // 初始化 arm_group
  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);
  arm.setPlanningTime(5.0);       // 加规划时间，更稳定
  arm.setNumPlanningAttempts(3);  // 多次尝试


  //初始化gripper_group
  auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);
  arm.setPlanningTime(5.0);       // 加规划时间，更稳定
  arm.setNumPlanningAttempts(3);  // 多次尝试



  // // 去预设姿态 pose1
  // arm.setStartStateToCurrentState();
  // arm.setNamedTarget("pose1");

  // gripper.setStartStateToCurrentState();
  // gripper.setNamedTarget("close");


  // // 规划
  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
  // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  // bool success_gripper1=(gripper.plan(plan_gripper)==moveit::core::MoveItErrorCode::SUCCESS);

  // // 执行
  // if (success1&&success_gripper1) {
  //   arm.execute(plan1);
  //   gripper.execute(plan_gripper);
  // }

  //  arm.setStartStateToCurrentState();
  //  arm.setNamedTarget("pose2");
  //  gripper.setStartStateToCurrentState();
  //  gripper.setNamedTarget("open");

  // // 规划
  // moveit::planning_interface::MoveGroupInterface::Plan plan2;
  // moveit::planning_interface::MoveGroupInterface::Plan plan_gripper1;
  // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  // bool success_gripper2=(gripper.plan(plan_gripper1)==moveit::core::MoveItErrorCode::SUCCESS);


  // // 执行
  // if (success2&&success_gripper2) {
  //   arm.execute(plan2);
  //   gripper.execute(plan_gripper1);

  // }
  //----------------------------------------------------------------------------------------------------
  //joint goal
  // std::vector<double> joint={1.5,0.5,0.0,1.5,0.0,-0.7};

  // arm.setStartStateToCurrentState();
  // arm.setJointValueTarget(joint);


  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);


  // if(success1)
  // {
  //   arm.execute(plan1);
  // }
  //pose goal

  tf2::Quaternion q;

  q.setRPY(3.14,0,0);
  q=q.normalize();

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id="base_link";

  target_pose.pose.position.x=0.7;
  target_pose.pose.position.y=0.0;
  target_pose.pose.position.z=0.4;

  target_pose.pose.orientation.x=q.getX();
  target_pose.pose.orientation.y=q.getY();
  target_pose.pose.orientation.z=q.getZ();
  target_pose.pose.orientation.w=q.getW();


  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);


  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);


  if(success1)
  {
    arm.execute(plan1);
  }

  std::vector<geometry_msgs::msg::Pose> waypoint;

  geometry_msgs::msg::Pose pose1=arm.getCurrentPose().pose;
  pose1.position.z+=-0.2;
  waypoint.push_back(pose1);
  geometry_msgs::msg::Pose pose2=pose1;

  pose2.position.y +=0.2;
  waypoint.push_back(pose2);

  geometry_msgs::msg::Pose pose3=pose2;

  pose3.position.y +=0.2;

  pose3.position.z +=0.2;
  waypoint.push_back(pose3);



  

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction=arm.computeCartesianPath(waypoint,0.01,0.0,trajectory,true, nullptr);

  if(fraction==1)
  {

    arm.execute(trajectory);
  }
  // 等待运动结束
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 优雅关闭
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return 0;
}
