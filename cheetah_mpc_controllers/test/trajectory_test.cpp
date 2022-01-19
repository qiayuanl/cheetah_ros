//
// Created by qiayuan on 2022/1/18.
//

#include <ros/ros.h>
#include <ros/publisher.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_sender");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/traj", 100);
  ros::Rate loop_rate(10);

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("roll");
  traj.joint_names.push_back("pitch");
  traj.joint_names.push_back("yaw");
  traj.joint_names.push_back("x");
  traj.joint_names.push_back("y");
  traj.joint_names.push_back("z");
  traj.header.stamp = ros::Time(0.5);

  double acc_x = 7.5;
  double acc_z = 15;
  double acc_pitch_deg = 500.;
  double duration = 0.2;
  int points = 10;
  double dt = duration / static_cast<double>(points);
  for (int i = 1; i < points + 1; ++i)
  {
    double acc_pitch = acc_pitch_deg / 180. * M_PI;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    point.velocities.resize(6);
    point.time_from_start = ros::Duration(dt * i);
    point.velocities[1] = acc_pitch * i * dt;
    point.velocities[3] = acc_x * i * dt;
    point.velocities[5] = acc_z * i * dt;
    point.positions[1] = 1. / 2. * acc_pitch * (i * dt) * (i * dt);
    point.positions[3] = 1. / 2. * acc_x * (i * dt) * (i * dt);
    point.positions[5] = 0.056 + 1. / 2. * acc_z * (i * dt) * (i * dt);
    traj.points.push_back(point);
  }
  while (ros::ok())
  {
    //    traj.header.stamp = ros::Time::now();
    pub.publish(traj);
    loop_rate.sleep();
  }
  return 0;
}
