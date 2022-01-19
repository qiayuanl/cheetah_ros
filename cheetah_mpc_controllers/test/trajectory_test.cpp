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
  ros::Rate loop_rate(0.5);

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("roll");
  traj.joint_names.push_back("pitch");
  traj.joint_names.push_back("yaw");
  traj.joint_names.push_back("x");
  traj.joint_names.push_back("y");
  traj.joint_names.push_back("z");
  traj.header.stamp = ros::Time(0.5);

  double vel_x = 1.;
  double vel_z = 3.;
  double rate_pitch_deg = 100.;
  double duration = 0.15;
  int points = 10;
  double dt = duration / static_cast<double>(points);
  for (int i = 1; i < points + 1; ++i)
  {
    double rate_pitch = rate_pitch_deg / 180. * M_PI;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    point.velocities.resize(6);
    point.time_from_start = ros::Duration(dt * i);
    point.velocities[1] = rate_pitch;
    point.velocities[3] = vel_x;
    point.velocities[5] = vel_z;
    point.positions[1] = rate_pitch * i * dt;
    point.positions[3] = vel_x * i * dt;
    point.positions[5] = 0.1 + vel_z * i * dt;
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
