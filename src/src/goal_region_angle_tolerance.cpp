#include "goal_region_angle_tolerance.h"

GoalRegionAngleTolerance::GoalRegionAngleTolerance(const ob::SpaceInformationPtr &si,
                                                   double positionTolerance,
                                                   double angleTolerance)
    : ob::GoalSampleableRegion(si), state_(nullptr), positionTolerance_(positionTolerance), angleTolerance_(angleTolerance)
{
    type_ = ob::GOAL_STATE;
}

GoalRegionAngleTolerance::~GoalRegionAngleTolerance()
{
    if (state_ != nullptr)
        si_->freeState(state_);
}

double GoalRegionAngleTolerance::distanceGoal(const ob::State *st) const
{
    return si_->distance(st, state_);
}

void GoalRegionAngleTolerance::print(std::ostream &out) const
{
    out << "Goal state, threshold = " << threshold_ << ", memory address = " << this << ", state = " << std::endl;
    si_->printState(state_, out);
}

void GoalRegionAngleTolerance::sampleGoal(ob::State *st) const
{
    si_->copyState(st, state_);
}

unsigned int GoalRegionAngleTolerance::maxSampleCount() const
{
    return 1;
}

void GoalRegionAngleTolerance::setState(const ob::State *st)
{
    if (state_ != nullptr)
        si_->freeState(state_);
    state_ = si_->cloneState(st);
}

void GoalRegionAngleTolerance::setState(const ob::ScopedState<> &st)
{
    setState(st.get());
}

const ob::State *GoalRegionAngleTolerance::getState() const
{
    return state_;
}

ob::State *GoalRegionAngleTolerance::getState()
{
    return state_;
}

// ==========================

bool GoalRegionAngleTolerance::isSatisfied(const ob::State *state) const
{

    return isSatisfied(state, nullptr);
}

bool GoalRegionAngleTolerance::isSatisfied(const ob::State *st, double *distance) const
{
    const ob::SE2StateSpace::StateType *se2state = st->as<ob::SE2StateSpace::StateType>();
    const ob::SE2StateSpace::StateType *goalSe2State = state_->as<ob::SE2StateSpace::StateType>();

    double dx = se2state->getX() - goalSe2State->getX();
    double dy = se2state->getY() - goalSe2State->getY();
    double dist = std::sqrt(dx * dx + dy * dy);

    double dtheta = std::fabs(se2state->getYaw() - goalSe2State->getYaw());
    dtheta = std::min(dtheta, 2 * M_PI - dtheta);

    return dist <= positionTolerance_ && dtheta <= angleTolerance_;
}
