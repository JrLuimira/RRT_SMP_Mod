#ifndef RRT_SMP_HPP
#define RRT_SMP_HPP

#include <vector>
#include <cmath>
#include <tuple>
#include <iostream>
#include <random>
#include <utility>
#include <functional>
#include <numeric> 

#include "rclcpp/rclcpp.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include <pedsim_msgs/msg/line_obstacles.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

// Built by: Luigi Miranda

struct Point2D
{
    // This is to save walls config
    double x, y;
};

struct Pose2D
{ // This is for motion primitives
    double x, y, theta;
};

double area(double dt, double step, double vmax, double wmax);

// Relative Config between Xinit and Xgoal
std::tuple<double, double> getRelativeConfigINIT(nav_msgs::msg::Odometry::SharedPtr odom_data, std::vector<double> goal_point);
// Relative Config robot - Xppl
std::vector<double> calculateRelativeConfigPPL(std::vector<double> linear_vel,
                                                std::vector<double> angular_vel,
                                                std::vector<double> distances,
                                                std::vector<double> angles);
// Relative Config robot - walls
std::vector<std::tuple<double, double>> getRelativeConfigWalls();
// Get the average of Xwall and Thetawall
std::tuple<double, double> getWallDistances();
// Eq 2: Fuerza de atracción al objetivo
std::tuple<double, double> getGoalIntent(double dgr, double Omegag, double mG, double offsetG);
//!Motion Primitives vectors
std::vector <double> mp_linear_velocities;
std::vector <double> mp_angular_velocities;

// Extra: Velocidad relativa Robot - Humano
double getRelativeVelocity(std::vector<double> vector);
// Extra: Obtain relative position robot - agent (distance,angle)

//!Get Relative config of each person to robot INIT. ONLY ROBOT INIT POSITION. LINE 8
std::tuple<double,double> getRelativeConfig(std::vector<double> distances,std::vector<double> angles, double x, double y);

// Eq 3:
double getPeopleIntent(double Vpr, double dpr, double DeltaP);

// Eq 4 & Eq 5: Reglas de movimiento
std::tuple<double, double> getRuleIntent(double FP, double mR, double offsetR);

// Eq 6 & Eq 7: Cálculo de velocidad media
std::tuple<double, double> getVelocityMeans(double Fg, double Fp, double thetaG, double thetaP, double max_v, double max_w);

// Eq 8: Modelo Gaussiano
std::vector<double> mpDistribution(double mean, double sigma, int num_samples, double delta);
// Eq 8-2: MP of mpDistribution
std::vector<std::pair<double,double>> MPgeneration(double v, double omega, double T, double dt, double A);
// Transform the motions primitives to arcs
Pose2D convertVelocitytoArc(const Pose2D &startPose, double v, double omega, double dt = 1.0);
// Transform states to Pose2D

//! Recive los vectores y devuelve sus valores.



// Callback del AgentStates
void agentCallback(const pedsim_msgs::msg::AgentStates::SharedPtr msg);
// Callback de Odometry
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
// Posiciones de peatones


// Posiciones del robot e gazebo
// std::vector<std::tuple<double, double>> robot_positions_;
// Posiciones de los muros
void wallCallback(const pedsim_msgs::msg::LineObstacles::SharedPtr msg);
// Lista de los segmentos de los muros
//std::vector<std::pair<Point2D, Point2D>> walls_;
// Goal position
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
// rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_suscriber_;
// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_suscriber_;
// rclcpp::Subscription<pedsim_msgs::msg::LineObstacles>::SharedPtr wall_suscriber_;
// rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_suscriber_;

//!Gauss
//std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());


#endif // RRT_SMP_HPP