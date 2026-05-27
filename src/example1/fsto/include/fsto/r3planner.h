#ifndef R3PLANNER_H
#define R3PLANNER_H

#include "fsto/config.h"
#include "fsto/glbmap.h"

#include <memory>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

class DirectionalMotionValidator : public ompl::base::MotionValidator
{
public:
    DirectionalMotionValidator(const ompl::base::SpaceInformationPtr &si,
                               std::shared_ptr<const LocalPerceptionMap> mapPtr,
                               const Config &config,
                               const Eigen::Vector3d &startPos,
                               const Eigen::Vector3d &initVel);

    bool checkMotion(const ompl::base::State *s1,
                     const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1,
                     const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override;

private:
    std::shared_ptr<const LocalPerceptionMap> mapPtr_;
    Config config_;
    Eigen::Vector3d startPos_;
    Eigen::Vector3d initDir_;
    bool enableDirectionConstraint_;
    double cosMaxTurnAngle_;

    Eigen::Vector3d stateToEigen(const ompl::base::State *state) const;

    bool pointSafe(const Eigen::Vector3d &p) const;

    bool directionAllowed(const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &p2) const;
};

class R3Planner
{
public:
    R3Planner(const Config &conf, std::shared_ptr<const LocalPerceptionMap> mapPtr);

    double plan(const Eigen::Vector3d &s,
                const Eigen::Vector3d &g,
                std::vector<Eigen::Vector3d> &p,
                double timeout) const;

    double plan(const Eigen::Vector3d &s,
                const Eigen::Vector3d &initVel,
                const Eigen::Vector3d &g,
                std::vector<Eigen::Vector3d> &p,
                double timeout) const;

    double planOnce(const Eigen::Vector3d &s,
                    const Eigen::Vector3d &g,
                    std::vector<Eigen::Vector3d> &p) const;

    double planOnce(const Eigen::Vector3d &s,
                    const Eigen::Vector3d &initVel,
                    const Eigen::Vector3d &g,
                    std::vector<Eigen::Vector3d> &p) const;

private:
    Config config;
    std::shared_ptr<const LocalPerceptionMap> MapPtr;
};

#endif