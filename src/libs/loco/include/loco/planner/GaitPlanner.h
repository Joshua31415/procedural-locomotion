//
// Created by Dongho Kang on 02.05.23.
//

#ifndef PROCEDURAL_LOCOMOTION_GAITPLANNER_H
#define PROCEDURAL_LOCOMOTION_GAITPLANNER_H

#include "loco/planner/FootFallPattern.h"

namespace crl::loco {

/**
 * base class
 */
class GaitPlanner {
public:
    virtual ~GaitPlanner() = default;

    virtual PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const = 0;
};

/**
 * for quadrupedal characters
 */
class QuadrupedalGaitPlanner : public GaitPlanner {
public:
    ~QuadrupedalGaitPlanner() override = default;

    PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const {
        PeriodicGait pg;
        double tOffset = -0.0;
        pg.addSwingPhaseForLimb(robot->getLimbByName("hl"), 0 - tOffset, 0.5 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("fl"), 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("hr"), 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("fr"), 0 - tOffset, 0.5 + tOffset);
        pg.strideDuration = 1.25;
        return pg;
    }
};

/**
 * for bipedal characters
 */
class BipedalGaitPlanner : public GaitPlanner {
public:
    ~BipedalGaitPlanner() override = default;

    PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const {
        PeriodicGait pg;
        double tOffset = -0.0;
        //pg.addSwingPhaseForLimb(robot->getLimbByName("lFoot"), 0 - tOffset, 0.5 + tOffset);
        //pg.addSwingPhaseForLimb(robot->getLimbByName("rFoot"), 0.5 - tOffset, 1.0 + tOffset);
        // pg.addSwingPhaseForLimb(robot->getLimbByName("lLowerLeg"), 0 - tOffset, 0.5 + tOffset);
        // pg.addSwingPhaseForLimb(robot->getLimbByName("rLowerLeg"), 0.5 - tOffset, 1.0 + tOffset);
        //pg.addSwingPhaseForLimb(robot->getLimbByName("rLowerArm"), 0 - tOffset, 0.5 + tOffset);
        //pg.addSwingPhaseForLimb(robot->getLimbByName("lLowerArm"), 0.5 - tOffset, 1.0 + tOffset);
        //pg.addSwingPhaseForLimb(robot->getLimbByName("rUpperArm"), 0 - tOffset, 0.5 + tOffset);
        //pg.addSwingPhaseForLimb(robot->getLimbByName("lUpperArm"), 0.5 - tOffset, 1.0 + tOffset);
        pg.strideDuration = 1.25;
        return pg;
    }
};

}  // namespace crl::loco

#endif  //PROCEDURAL_LOCOMOTION_GAITPLANNER_H
