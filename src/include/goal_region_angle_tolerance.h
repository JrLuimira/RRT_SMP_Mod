#ifndef GOAL_REGION_ANGLE_TOLERANCE_H
#define GOAL_REGION_ANGLE_TOLERANCE_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "ompl/base/ScopedState.h"
#include <cmath> // For std::sqrt and std::fabs

namespace ob = ompl::base;

class GoalRegionAngleTolerance : public ob::GoalSampleableRegion
{
public:
    GoalRegionAngleTolerance(const ob::SpaceInformationPtr &si,
                             double positionTolerance,
                             double angleTolerance);
    ~GoalRegionAngleTolerance() override;

    void sampleGoal(ob::State *st) const override;

    unsigned int maxSampleCount() const override;

    virtual double distanceGoal(const ob::State *st) const override;

    void print(std::ostream &out = std::cout) const override;

    void setState(const ob::State *st);

    void setState(const ob::ScopedState<> &st);

    const ob::State *getState() const;

    ob::State *getState();

    // =======================================================

    // Custom goal satisfaction check
    bool isSatisfied(const ob::State *state) const override;
    bool isSatisfied(const ob::State *st, double *distance) const override;

protected:
    ob::State *state_;

private:
    double positionTolerance_;
    double angleTolerance_;
};

#endif // GOAL_REGION_ANGLE_TOLERANCE_H