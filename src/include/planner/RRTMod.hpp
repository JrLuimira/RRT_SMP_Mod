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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_MOD_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_MOD_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <planner/RRT_SMP.hpp>

#include <nav_msgs/msg/path.hpp>


namespace ompl
{
    namespace geometric
    {
        class RRTMod : public base::Planner
        {
        public:
            RRTMod(const base::SpaceInformationPtr &si,
                   std::vector<double> &linear_vel_vector,
                   std::vector<double> &angular_vel_vector,
                   std::vector<double> &distances_vector,
                   std::vector<double> &angle_vector,
                   std::vector<double> &goal_point,
                   nav_msgs::msg::Odometry::SharedPtr odom_data_,

                   bool addIntermediateStates = false);

            ~RRTMod() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                base::State *state{nullptr};

                Motion *parent{nullptr};
            };

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;

            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            double goalBias_{.05};

            double maxDistance_{0.};

            // ompl::base::SE2StateSpace::StateType *simulateArcStep(
            //     const ompl::base::SE2StateSpace::StateType *from,
            //     const ompl::base::SE2StateSpace::StateType *to,

            //     double v, double w, double dt,
            //     const ompl::base::SpaceInformationPtr &si);

            nav_msgs::msg::Path *simulateArcStep(
                const ompl::base::State *from,
                const ompl::base::State *to,
                double v, double w, double dt);

            ompl::base::State *TransformPointsToStates(
                double x,
                double y,
                const ompl::base::SpaceInformationPtr &si);

            Pose2D stateToPose2D(const ompl::base::State *state);

            //! Declaring the vectors of SMP
            std::vector<double> &linear_vel_vector_;
            std::vector<double> &angular_vel_vector_;
            std::vector<double> &distances_vector_;
            std::vector<double> &angle_vector_;
            std::vector<double> &goal_point_;
            nav_msgs::msg::Odometry::SharedPtr odom_data_;
            bool addIntermediateStates_;

            //! Declare the vector of forces and relative configs the code will be calculating
            std::tuple<double, double> getInitialConfig_value;
            std::tuple<double, double> getRelativeConfig_value;
            std::vector<double> pederastianVector_value;
            double getPeopleIntent_value;
            std::tuple<double, double> getVelocityMeans_value;
            std::vector<double> forces;
            std::tuple<double, double> getGoalIntent_value;
            nav_msgs::msg::Path *valid_arc_points;
            // For MP
            std::vector<double> linear_gaussian;
            std::vector<double> omega_gaussian;


            Pose2D arc_convert;
            Pose2D State2Pose;

            double x;
            double y;
            double theta;
            RNG rng_;

            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif