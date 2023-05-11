#pragma once

#include <loco/controller/LocomotionController.h>
#include <loco/kinematics/IK_Solver.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>

namespace crl::loco {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    std::shared_ptr<LeggedRobot> robot;
    std::shared_ptr<IK_Solver> ikSolver = nullptr;

    double cycle = 0;
    P3D rFootTarget;
    P3D rFootFinal;
    P3D rFootInitial;
    P3D lFootTarget;
    P3D lFootFinal;
    P3D lFootInitial;
    bool rFootSet = false;
    bool lFootSet = false;
    bool lLocked = false;
    bool rLocked = false;

public:
    /**
     * constructor
     */
    KinematicTrackingController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);
    }

    /**
     * destructor
     */
    ~KinematicTrackingController(void) override = default;

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);

        robot->setRootState(targetPos, targetOrientation);

        // std::cout << cycle << "\n";
        auto lFoot = robot->getLimb(1);
        auto rFoot = robot->getLimb(0);
        
        if (lLocked) {
            // stance phase
            cycle += dt;
            if (cycle >= 1.0) {
                lLocked = false;
                lFootInitial = lFoot->getEEWorldPos();
                cycle = cycle - int(cycle);
            }
        } else {
            // swing phase
            double frac = std::min(std::max(2.0 * cycle, 0.0), 1.0);
            lFootFinal = lFoot->limbRoot->getWorldCoordinates() + lFoot->defaultEEOffsetWorld;
            lFootTarget = lFootInitial * (1.0 - frac) + lFootFinal * frac;


            cycle += dt;
            if (cycle >= 0.5) {
                lFootTarget = lFoot->getEEWorldPos();
                lLocked = true;
            }
        }

        ikSolver->addEndEffectorTarget(lFoot->eeRB, lFoot->ee->endEffectorOffset, lFootTarget);
        ikSolver->solve();
    }

    void advanceInTime(double dt) override {
        planner->advanceInTime(dt);
    }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco