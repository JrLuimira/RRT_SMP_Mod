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
  
 #ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
 #define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
  
 #include "ompl/datastructures/NearestNeighbors.h"
 #include "ompl/geometric/planners/PlannerIncludes.h"
  
 namespace ompl
 {
     namespace geometric
     {
         class RRT : public base::Planner
         {
         public:
             RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);
  
             ~RRT() override;
  
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
  
             bool addIntermediateStates_;
  
             RNG rng_;
  
             Motion *lastGoalMotion_{nullptr};
         };
     }
 }
  
 #endif