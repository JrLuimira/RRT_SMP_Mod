/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
// Modified by Luigi Miranda

// #include "ompl/geometric/planners/rrt/RRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <planner/RRTMod.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ompl::geometric::RRTMod::RRTMod(const base::SpaceInformationPtr &si,
                                std::vector<double> &linear_vel_vector,
                                std::vector<double> &angular_vel_vector,
                                std::vector<double> &distances_vector,
                                std::vector<double> &angle_vector,
                                std::vector<double> &goal_point,
                                nav_msgs::msg::Odometry::SharedPtr odom_data,
                                bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "RRTintermediate" : "RRTMod"),
      linear_vel_vector_(linear_vel_vector),
      angular_vel_vector_(angular_vel_vector),
      distances_vector_(distances_vector),
      angle_vector_(angle_vector),
      goal_point_(goal_point),
      odom_data_(odom_data)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTMod::setRange, &RRTMod::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTMod::setGoalBias, &RRTMod::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTMod::setIntermediateStates, &RRTMod::getIntermediateStates,
                                "0,1");
    addIntermediateStates_ = addIntermediateStates;
}

// Destructor definition
ompl::geometric::RRTMod::~RRTMod()
{
    freeMemory();
}

void ompl::geometric::RRTMod::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRTMod::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return distanceFunction(a, b); });
}

void ompl::geometric::RRTMod::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}
//! Creating the Arc
nav_msgs::msg::Path *ompl::geometric::RRTMod::simulateArcStep(
    const ompl::base::State *from,
    const ompl::base::State *to,
    double v, double w, double dt)
{

    //! Motion primitives array B = [[vi,wi,]...]
    // p_linear_velocities = {-8.0, -4.0, -1.0, 0.0, 1.0, 4.0, 8.0};
    // mp_angular_velocities = {-4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0};

    if (!from || !to)
    {
        OMPL_ERROR("simulateArcStep: One of the states is null.");
        return nullptr;
    }

    // Asumimos que el estado es directamente un SE2StateSpace::StateType
    const auto *fromSE2 = from->as<ompl::base::SE2StateSpace::StateType>();
    const auto *toSE2 = to->as<ompl::base::SE2StateSpace::StateType>();

    if (!fromSE2 || !toSE2)
    {
        OMPL_ERROR("simulateArcStep: Could not cast states to SE2StateSpace::StateType.");
        return nullptr;
    }

    // Default or fixed number of points
    double max_linear_vel = *std::max_element(mp_linear_velocities.begin(), mp_linear_velocities.end());
    double max_angular = *std::max_element(mp_angular_velocities.begin(), mp_angular_velocities.end());
    int num_points = 24;
    
    double A = area(dt, num_points, max_linear_vel, max_angular); //! NOT USING THIS VALUE YET

    double x = fromSE2->getX();
    double y = fromSE2->getY();

    double theta = fromSE2->getYaw();
    auto *path_msg = new nav_msgs::msg::Path();
    path_msg->poses.reserve(num_points);

    linear_gaussian = mpDistribution(max_linear_vel, 0.2, num_points, 0.1); // Linear distribution
    omega_gaussian = mpDistribution(max_angular, 0.2, num_points, 0.1);     // Angular distribution

    for (double linear : linear_gaussian)
    {
        for (double angular : omega_gaussian)
        {
            for (int i = 0.0; i <= 1.0; i += dt)
            {
                for (double t = 0.0; t < 1.0; t += dt)
                {
                    if (std::abs(angular) < 1e-6)
                    {
                        x += linear * dt * std::cos(theta);
                        y += linear * dt * std::sin(theta);
                    }
                    else
                    {
                        double r = linear / angular;
                        x += r * (std::sin(theta + angular * dt) - std::sin(theta));
                        y += -r * (std::cos(theta + angular * dt) - std::cos(theta));
                        theta += angular * dt;
                    }
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);
                    pose.pose.orientation = tf2::toMsg(q);

                    path_msg->poses.push_back(pose);

                    // Simular siguiente paso
                    // x += v * std::cos(theta) * dt;
                    // y += v * std::sin(theta) * dt;
                    // theta += w * dt;
                    OMPL_INFORM("x: %f", x);
                    OMPL_INFORM("y: %f", y);
                    OMPL_INFORM("theta: %f", theta);
                }
            }
        }
    }

    return path_msg;
}

//! Create a function that convert doubleX, doubleY into space points
ompl::base::State *ompl::geometric::RRTMod::TransformPointsToStates(
    double x,
    double y,
    const ompl::base::SpaceInformationPtr &si)
{
    if (!si || !si->isSetup())
    {
        OMPL_ERROR("SpaceInformation no está inicializado.");
        return nullptr;
    }

    auto space = si->getStateSpace();
    auto *state = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
    state->setXY(x, y);
    return state;
}

ompl::base::PlannerStatus ompl::geometric::RRTMod::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    //! NEED TO GET THE INIT POS AND GOAL POS

    getInitialConfig_value = getRelativeConfig(distances_vector_, angle_vector_, -4.0, 5.0);
    double d_Initial = std::get<0>(getInitialConfig_value);
    double theta_Initial = std::get<1>(getInitialConfig_value);
    // OMPL_INFORM("Initial distance is: %f", d_Initial);
    // OMPL_INFORM("Initial angle is: %f", theta_Initial);

    //! Here the RRT_SMP starts
    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        //! SMP: Get relative config

        // Initial config

        // Current config
        // getRelativeConfig_value = getRelativeConfig(1.0, 2.0, -4.0, 5.0);
        // double d_config = std::get<0>(getRelativeConfig_value);
        // double theta_config = std::get<1>(getRelativeConfig_value);
        // OMPL_INFORM("Current distance is: %f", d_config);
        // OMPL_INFORM("Current angle is: %f", theta_config);

        // FG force
        getGoalIntent_value = getGoalIntent(d_Initial, theta_Initial, 2, 0.0);
        double FG_value = std::get<0>(getGoalIntent_value);
        double ThetaG_value = std::get<1>(getGoalIntent_value);
        // OMPL_INFORM("Force Goal is: %f", FG_value);
        // OMPL_INFORM("Angle force is: %f", ThetaG_value);
        // OMPL_INFORM("Antes de guardas las fuerzas");
        /* find closest state in the tree */
        // OMPL_INFORM("ANTES DE MOTION");
        Motion *nmotion = nn_->nearest(rmotion);

        /* find state to add */

        //! En esta parte da error: I dont know why

        //! Here is validated all the peredastian config available
        //! For cycle, line 6 code
        // OMPL_INFORM("DESPUES DE BASE");

        pederastianVector_value = calculateRelativeConfigPPL(linear_vel_vector_, angular_vel_vector_, distances_vector_, angle_vector_);

        // OMPL_INFORM("DESPUES DE llamar a la funcion de pederastian");

        double total_vel_lin = pederastianVector_value[0];
        // OMPL_INFORM("Primer valor");
        double total_distance = pederastianVector_value[2];
        // OMPL_INFORM("Segundo valor");
        double total_angle = pederastianVector_value[3];
        // OMPL_INFORM("------Now the pederastian values-------");
        // OMPL_INFORM("Linear velocity: %f", total_vel_lin);
        // OMPL_INFORM("Distance: %f", total_distance);
        // OMPL_INFORM("Angle: %f", total_angle);

        getPeopleIntent_value = getPeopleIntent(total_vel_lin, total_distance, total_angle); // FP force

        getVelocityMeans_value = getVelocityMeans(FG_value, getPeopleIntent_value, ThetaG_value, total_angle);
        double v_mean = std::get<0>(getVelocityMeans_value);
        double w_mean = std::get<1>(getVelocityMeans_value);
        // OMPL_INFORM("--------- The velocities of SMP --------------");
        // OMPL_INFORM("Velocity mean: %f", v_mean);
        // OMPL_INFORM("Angular mean: %f", w_mean);

        //  nmotion->state->as<ompl::base::SE2StateSpace::StateType>(),
        //     rstate->as<ompl::base::SE2StateSpace::StateType>(),
        std::string spaceName = si_->getStateSpace()->getName();
        OMPL_INFORM("State space name: %s", spaceName.c_str());
        valid_arc_points = simulateArcStep(
            nmotion->state,
            rstate,
            v_mean,
            w_mean,
            1.0);

        // double d = si_->distance(nmotion->state, rstate); // current

        // if (d > maxDistance_)
        // {
        //     si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
        //     dstate = xstate;
        // }

        //! This part is for checking collision of the curve
        // Iterate over each pose in the simulated arc path
        for (const auto &pose_stamped : valid_arc_points->poses)
        {

            OMPL_INFORM("Si corre ");

            // Convert pose_stamped to SE2 state
            double aux1 = pose_stamped.pose.position.x;
            double aux2 = pose_stamped.pose.position.y;
            // double aux3 = tf2::getYaw(pose_stamped.pose.orientation);

            ompl::base::State *new_intermediate_state_curve = TransformPointsToStates(aux1, aux2, si_);

            if (si_->checkMotion(nmotion->state, new_intermediate_state_curve))
            {
                if (addIntermediateStates_)
                {

                    std::vector<base::State *> states;
                    const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, new_intermediate_state_curve);

                    if (si_->getMotionStates(nmotion->state, new_intermediate_state_curve, states, count, true, true))
                        si_->freeState(states[0]);
                    for (std::size_t i = 1; i < states.size(); ++i)
                    {
                        auto *motion = new Motion;
                        motion->state = states[i];
                        motion->parent = nmotion;

                        nn_->add(motion);

                        nmotion = motion;
                    }
                }
                else
                {
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, new_intermediate_state_curve);
                    motion->parent = nmotion;
                    nn_->add(motion);
                    nmotion = motion;
                }

                double dist = 0.0;
                bool sat = goal->isSatisfied(nmotion->state, &dist);
                if (sat)
                {
                    approxdif = dist;
                    solution = nmotion;

                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = nmotion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
    //! Free the memory
    // delete valid_arc_points;
}

void ompl::geometric::RRTMod::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
