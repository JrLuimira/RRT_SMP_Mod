#include <tuple>
#include "planner/RRT_SMP.hpp"



//! EQUATIONS
// Eq 1: Size of branches area
double area(double dt, double step, double vmax, double wmax)
 {
     return 2 * std::pow(dt * vmax * step,2.0) * std::cos(dt *wmax*step);
 }

// Extra: Obtain the relative config between init and goal

std::tuple<double, double> getRelativeConfigINIT(nav_msgs::msg::Odometry::SharedPtr odom_data, std::vector<double> goal_point)
{
    double odom_x; double odom_y; double odom_z; double goal_x; double goal_y; double goal_z;

    double dGr = std::sqrt(std::pow(goal_x - odom_x, 2) + std::pow(goal_y - odom_y, 2));

    double thetaGr = std::atan2(goal_y - odom_y, goal_x - odom_x);

    return std::make_tuple(dGr, thetaGr);
}

// Eq 2: Get the Fg force and orientation
std::tuple<double, double> getGoalIntent(double dGr, double thetaGr, double mG, double offsetG)
{
    double Fgr = (mG * dGr) - offsetG;
    // Fgr uses the same vector of dGr, so they have the same direction
    return std::make_tuple(Fgr, thetaGr);
}
//! LINE 8
std::tuple<double,double> getRelativeConfig(std::vector<double> distances,std::vector<double> angles, double robot_x, double robot_y)
{
    double distance_final = 0;
    double angle_final = 0;
    for(int i=0; i<distances.size(); i++)
    {
        double distance_robot = std::hypot(robot_x,robot_y);
        double distance_person = distances[i];
        double aux = std::abs(distance_person - distance_robot);
        distance_final += aux; 

        double aux_angle = std::abs(std::atan2(robot_y,robot_x ) - angles[i]*180/3.14);
        
    }

    return std::make_tuple(distance_final,angle_final);
}

// Extra: Linea 6 to 9 pseudocode
std::vector<double> calculateRelativeConfigPPL(
    std::vector<double> linear_vel,
    std::vector<double> angular_vel,
    std::vector<double> distances,
    std::vector<double> angles)
{
    std::vector<double> configs;
    //Obtain all the element sum in configs vect. In this format, configs = {linear_vel,angular_vel,distances,angles}
    double linear_sum = accumulate(linear_vel.begin(), linear_vel.end(),0.0);
    configs.push_back(linear_sum);
    double angular_sum = accumulate(angular_vel.begin(), angular_vel.end(),0.0);
    configs.push_back(angular_sum);
    double distances_sum = accumulate(distances.begin(), distances.end(),0.0);
    configs.push_back(distances_sum);
    double angles_sum = accumulate(angles.begin(), angles.end(),0.0);
    configs.push_back(angles_sum);

    
    return configs;
}


// Eq 3: Obtain the Fp force
double getPeopleIntent(double Vpr, double dpr, double DeltaP)
{
    double numerator = 60 * Vpr - 100.0;
    double denominator = 1.0 + std::abs(dpr - DeltaP);
    double FP = numerator / denominator;
    return FP;
}


// Eq 4 & 5: Obtain the movement
// std::tuple<double, double> getRuleIntent(double FP, double mR, double offsetR)
// {
//     auto [dRr, thetaRr] = getWallDistances();
//     // Eq 4: The force due to walls
//     double FR_av = (mR * dRr) - offsetR;
//     // Eq 5:
//     double FR = FR_av * (1.0 - FP);
//     return std::make_tuple(FR, thetaRr);
// }



std::tuple<double, double> getVelocityMeans(double Fg, double Fp, double thetaG, double thetaP, double max_v, double max_w)
{
    //Eq 6 & 7
    double Mg = 1, Mp = 1, nF = 2;

    double denominator_v_goal = 1.0 + std::exp(-5.0 * Fg /Mg)*std::cos(thetaG); 
    double denominator_w_goal = 1.0 + std::exp(-5.0 * Fg /Mg)*std::sin(thetaG);
    double denominator_v_person = 1.0 + std::exp(-5.0 * Fp /Mp)*std::cos(thetaP); 
    double denominator_w_person = 1.0 + std::exp(-5.0 * Fp /Mp)*std::sin(thetaP); 
    double numerator_goal = Mg/nF;
    double numerator_person = Mp/nF;

    double v_mean = (max_v  / nF)*(1/denominator_v_goal + 1/denominator_v_goal);
    double w_mean = (max_w/nF)*(1/denominator_w_goal + 1/denominator_w_person);

    return std::make_tuple(v_mean, w_mean);
}

//! LINEA 16 TO 25 ARE FOR MOTION PRIMITIVES GENERATION
std::vector<double> mpDistribution(double mean, double sigma, int num_samples, double delta)
{
    std::vector<double> samples;
    int half = num_samples/2;
    
    for (int i = -half; i <= half; ++i) {

        samples.push_back(mean + i * delta);
    }
    return samples;
}

std::vector<std::pair<double,double>> MPgeneration(double v, double omega, double T, double dt,double A) // This "A" variable derives from "area" function
{
    std::vector<std::pair<double,double>> trajectory;
    //In this point the initial point is suppose as the origin.
    double x=0.0, y=0.0, theta=0.0;

    for(double t=0.0; t<T; t+=dt)
    {
        if(std::abs(omega<1e-6))
        {
            x+=v*dt*std::cos(theta);
            y+=v*dt*std::sin(theta);

        }else{
            double r=v/omega;

            x+=r*(std::sin(theta+(omega*dt))-std::sin(theta));
            y+=r*(std::cos(theta+(omega*dt))-std::cos(theta));

            theta+=omega*dt;
        }
        trajectory.emplace_back(x,y);
    }
    return trajectory;
}


Pose2D convertVelocitytoArc(const Pose2D &startPose, double v, double omega, double dt)
{
    Pose2D newPose;

    if (std::abs(omega) < 1e-5)
    {
        newPose.x = startPose.x + v * dt * cos(startPose.theta);
        newPose.y = startPose.y + v * dt * sin(startPose.theta);
        newPose.theta = startPose.theta;
    }
    else
    {
        double R = v / omega;
        newPose.x = startPose.x + R * (sin(startPose.theta + omega * dt) - sin(startPose.theta));
        newPose.y = startPose.y + R * (cos(startPose.theta + omega * dt) - cos(startPose.theta));
        newPose.theta = startPose.theta + omega * dt;
    }
    return newPose;
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<rrt_smp::RRT_SMP>(1.0, 60.0, 1.0 ,1.0, 30.0, 1.0, 30.0, 1.0, 0.0, 0.0,5.0, 6.0, 60.0, 1.0);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }