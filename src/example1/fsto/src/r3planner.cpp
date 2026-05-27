#include "fsto/r3planner.h"

#include <cmath>
#include <algorithm>

using namespace std;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

DirectionalMotionValidator::DirectionalMotionValidator(
    const ob::SpaceInformationPtr &si,
    std::shared_ptr<const LocalPerceptionMap> mapPtr,
    const Config &config,
    const Eigen::Vector3d &startPos,
    const Eigen::Vector3d &initVel)
    : ob::MotionValidator(si),
      mapPtr_(mapPtr),
      config_(config),
      startPos_(startPos),
      initDir_(Eigen::Vector3d::Zero()),
      enableDirectionConstraint_(false),
      cosMaxTurnAngle_(std::cos(config.r3_max_initial_turn_angle * M_PI / 180.0))
{
    Eigen::Vector3d v_xy(initVel(0), initVel(1), 0.0);

    if (v_xy.norm() > config_.r3_direction_min_vel)
    {
        initDir_ = v_xy.normalized();
        enableDirectionConstraint_ = true;
    }
}

Eigen::Vector3d DirectionalMotionValidator::stateToEigen(
    const ob::State *state) const
{
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    return Eigen::Vector3d((*pos)[0], (*pos)[1], (*pos)[2]);
}

bool DirectionalMotionValidator::pointSafe(const Eigen::Vector3d &p) const
{
    return mapPtr_->safeQuery(p, config_.r3SafeRadius);
}

bool DirectionalMotionValidator::directionAllowed(const Eigen::Vector3d &p1,
                                                  const Eigen::Vector3d &p2) const
{
    if (!enableDirectionConstraint_)
    {
        return true;
    }

    // 只约束起点附近的边，不约束整条路径。
    const double d1 = (p1 - startPos_).norm();
    const double d2 = (p2 - startPos_).norm();

    if (std::min(d1, d2) > config_.r3_direction_constraint_radius)
    {
        return true;
    }

    Eigen::Vector3d edge = p2 - p1;
    edge(2) = 0.0;

    if (edge.norm() < 1e-3)
    {
        return true;
    }

    edge.normalize();

    const double cos_angle = edge.dot(initDir_);

    return cos_angle >= cosMaxTurnAngle_;
}

bool DirectionalMotionValidator::checkMotion(const ob::State *s1,
                                             const ob::State *s2) const
{
    const Eigen::Vector3d p1 = stateToEigen(s1);
    const Eigen::Vector3d p2 = stateToEigen(s2);

    if (!directionAllowed(p1, p2))
    {
        return false;
    }

    const Eigen::Vector3d diff = p2 - p1;
    const double len = diff.norm();

    if (len < 1e-6)
    {
        return pointSafe(p1) && pointSafe(p2);
    }

    const double resolution = std::max(1e-3, config_.spatialResolution);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / resolution)));

    for (int i = 0; i <= steps; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::Vector3d p = p1 + alpha * diff;

        if (!pointSafe(p))
        {
            return false;
        }
    }

    return true;
}

bool DirectionalMotionValidator::checkMotion(
    const ob::State *s1,
    const ob::State *s2,
    std::pair<ob::State *, double> &lastValid) const
{
    const Eigen::Vector3d p1 = stateToEigen(s1);
    const Eigen::Vector3d p2 = stateToEigen(s2);

    const Eigen::Vector3d diff = p2 - p1;
    const double len = diff.norm();

    const double resolution = std::max(1e-3, config_.spatialResolution);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / resolution)));

    double lastValidFraction = 0.0;

    for (int i = 0; i <= steps; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::Vector3d p = p1 + alpha * diff;

        if (!pointSafe(p))
        {
            if (lastValid.first != nullptr)
            {
                ob::State *tmp = si_->allocState();
                auto *rv = tmp->as<ob::RealVectorStateSpace::StateType>();

                const Eigen::Vector3d p_valid = p1 + lastValidFraction * diff;
                (*rv)[0] = p_valid(0);
                (*rv)[1] = p_valid(1);
                (*rv)[2] = p_valid(2);

                si_->copyState(lastValid.first, tmp);
                si_->freeState(tmp);
            }

            lastValid.second = lastValidFraction;
            return false;
        }

        lastValidFraction = alpha;
    }

    if (!directionAllowed(p1, p2))
    {
        if (lastValid.first != nullptr)
        {
            si_->copyState(lastValid.first, s1);
        }

        lastValid.second = 0.0;
        return false;
    }

    if (lastValid.first != nullptr)
    {
        si_->copyState(lastValid.first, s2);
    }

    lastValid.second = 1.0;
    return true;
}

R3Planner::R3Planner(const Config &conf,
                     std::shared_ptr<const LocalPerceptionMap> mapPtr)
    : config(conf), MapPtr(mapPtr)
{
}

double R3Planner::plan(const Vector3d &s,
                       const Vector3d &g,
                       vector<Vector3d> &p,
                       double timeout) const
{
    return plan(s, Eigen::Vector3d::Zero(), g, p, timeout);
}

double R3Planner::plan(const Vector3d &s,
                       const Vector3d &initVel,
                       const Vector3d &g,
                       vector<Vector3d> &p,
                       double timeout) const
{
    p.clear();

    if (!MapPtr->safeQuery(s, config.r3SafeRadius))
    {
        ROS_WARN("[R3Planner] Start is unsafe.");
        return INFINITY;
    }

    if (!MapPtr->safeQuery(g, config.r3SafeRadius))
    {
        ROS_WARN("[R3Planner] Goal is unsafe.");
        return INFINITY;
    }

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; i++)
    {
        bounds.setLow(i, std::max(config.r3Bound[i * 2 + 0],
                                  std::min(s(i), g(i)) - config.r3SafeRadius * 7.0));

        bounds.setHigh(i, std::min(config.r3Bound[i * 2 + 1],
                                   std::max(s(i), g(i)) + config.r3SafeRadius * 7.0));
    }
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));

    si->setStateValidityChecker(
        [&](const ob::State *state) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
            Vector3d position((*pos)[0], (*pos)[1], (*pos)[2]);
            return this->MapPtr->safeQuery(position, config.r3SafeRadius);
        });

    si->setMotionValidator(std::make_shared<DirectionalMotionValidator>(
        si, this->MapPtr, this->config, s, initVel));

    si->setup();

    ob::ScopedState<> start(space), goal(space);
    start[0] = s(0);
    start[1] = s(1);
    start[2] = s(2);

    goal[0] = g(0);
    goal[1] = g(1);
    goal[2] = g(2);

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    auto planner(std::make_shared<og::InformedRRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved;
    solved = planner->ob::Planner::solve(timeout);

    double cost = INFINITY;

    if (solved)
    {
        const og::PathGeometric path_ =
            og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));

        for (size_t i = 0; i < path_.getStateCount(); i++)
        {
            auto state = path_.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
            p.emplace_back(state[0], state[1], state[2]);
        }

        cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
    }

    return cost;
}

double R3Planner::planOnce(const Vector3d &s,
                           const Vector3d &g,
                           vector<Vector3d> &p) const
{
    return plan(s, Eigen::Vector3d::Zero(), g, p, config.searchDuration);
}

double R3Planner::planOnce(const Vector3d &s,
                           const Vector3d &initVel,
                           const Vector3d &g,
                           vector<Vector3d> &p) const
{
    return plan(s, initVel, g, p, config.searchDuration);
}