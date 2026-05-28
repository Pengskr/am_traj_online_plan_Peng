#include "fsto/r3planner.h"

#include <cmath>
#include <algorithm>

using namespace std;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

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
    p.clear();

    if (!MapPtr->safeQuery(s, config.r3SafeRadius))
    {
        ROS_WARN("[R3Planner] Start (%.2f,%.2f,%.2f) is unsafe.", s(0), s(1), s(2));
        return INFINITY;
    }

    if (!MapPtr->safeQuery(g, config.r3SafeRadius))
    {
        ROS_WARN("[R3Planner] Goal (%.2f,%.2f,%.2f) is unsafe.", g(0), g(1), g(2));
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

    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));

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

    // ============================================================
    // Cost stagnation early stop
    // ============================================================
    const double check_interval =
        std::max(1e-3, config.r3_cost_check_interval);

    const double stagnation_time =
        std::max(check_interval, config.r3_cost_stagnation_time);

    const double improve_threshold =
        std::max(0.0, config.r3_cost_improvement_threshold);

    double elapsed = 0.0;
    double lastImprovedTime = 0.0;
    double bestCost = std::numeric_limits<double>::infinity();

    bool hasSolution = false;

    while (elapsed < timeout)
    {
        const double dt = std::min(check_interval, timeout - elapsed);

        ob::PlannerStatus status = planner->ob::Planner::solve(dt);

        elapsed += dt;

        if (!status)
        {
            continue;
        }

        hasSolution = true;

        const double currentCost =
            pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();

        if (!std::isfinite(bestCost) ||
            bestCost - currentCost > improve_threshold)
        {
            bestCost = currentCost;
            lastImprovedTime = elapsed;
        }

        if (elapsed - lastImprovedTime >= stagnation_time)
        {
            ROS_WARN("[R3Planner] Early stop by cost stagnation. "
                     "elapsed=%.3f / %.3f, best_cost=%.3f",
                     elapsed,
                     timeout,
                     bestCost);
            break;
        }
    }

    double cost = INFINITY;

    if (hasSolution)
    {
        const og::PathGeometric path_ =
            og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));

        for (size_t i = 0; i < path_.getStateCount(); i++)
        {
            auto state = path_.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
            p.emplace_back(state[0], state[1], state[2]);
        }

        cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();

        ROS_WARN("[R3Planner] Solved. elapsed=%.3f / %.3f, cost=%.3f, state_count=%lu",
                 elapsed,
                 timeout,
                 cost,
                 path_.getStateCount());
    }

    return cost;
}

double R3Planner::planOnce(const Vector3d &s,
                           const Vector3d &g,
                           vector<Vector3d> &p) const
{
    return plan(s, g, p, config.searchDuration);
}