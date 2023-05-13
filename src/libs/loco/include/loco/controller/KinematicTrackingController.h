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

    double t = 0;
    double cycleLength = 1;
    double stanceStart = 0.0;
    double stanceEnd = 0.5;
    double swingStart = 0.5;
    double swingEnd = 1.0;

    std::array<P3D, 2> stanceTargets;
    std::array<P3D, 2> swingTargets;
    std::array<Trajectory3D, 2> swingTrajectories;
    std::array<std::array<int, 4>, 2> footJointIndices;

    enum Phase {Stance, Swing};
    int its = 0;

public:
    /**
     * constructor
     */
    KinematicTrackingController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);

        for (int i = 0; i < 2; ++i) {
            P3D p = robot->getLimb(i)->getEEWorldPos();
            stanceTargets[i] = p;
            swingTrajectories[i].addKnot(0, V3D(p));
            swingTrajectories[i].addKnot(1, V3D(p));
        }

        std::array<char *, 4> lJoints = {"lHip_1", "lKnee", "lAnkle_1", "lToeJoint"};
        std::array<char *, 4> rJoints = {"rHip_1", "rKnee", "rAnkle_1", "rToeJoint"};
        for (int i = 0; i < 4; ++i) {
            int lIdx = robot->getJointIndex(lJoints[i]);
            int rIdx = robot->getJointIndex(rJoints[i]);
            footJointIndices[0][i] = rIdx;
            footJointIndices[1][i] = lIdx;
        }
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

        t += dt;
        // use the toe as the targeted endeffector in the stance phase
        // use the heel as the targeted endeffector in the swing phase
        // idx i == 0 <-> right leg
        for (int i = 0; i < 2; ++i) {
            Phase currentPhase = getPhase(i, t);
            Phase nextPhase = getPhase(i, t + dt);

            if (currentPhase == Phase::Stance && nextPhase == Phase::Stance) {
                // foot stays still during stance phase
                setStanceTarget(i);
                ikSolver->solve();
                fixToeAngle(i);
            } else if (currentPhase == Phase::Stance && nextPhase == Phase::Swing) {
                computeSwingTrajectory(i);
                setSwingTarget(i);
                ikSolver->solve();
            } else if (currentPhase == Phase::Swing && nextPhase == Phase::Swing) {
                setSwingTarget(i);
                ikSolver->solve();
            } else if (currentPhase == Phase::Swing && nextPhase == Phase::Stance) {
                // foot stays still during stance phase
                computeStanceTarget(i);
                setStanceTarget(i);
                ikSolver->solve();
                fixToeAngle(i);
            }
        }

    }

    void computeSwingTrajectory(int i) {
        auto foot = robot->getLimb(i);

        swingTrajectories[i].clear();
        P3D pStart = foot->getEEWorldPos();
        V3D offset(pStart, foot->limbRoot->getWorldCoordinates() + foot->defaultEEOffsetWorld);
        P3D pEnd = pStart + offset * 2;
        swingTrajectories[i].addKnot(0, V3D(pStart));
        swingTrajectories[i].addKnot(1, V3D(pEnd));
    }

    void setSwingTarget(int i) {
        auto foot = robot->getLimb(i);
        double cyclePercent = getCyclePercent(i, t);
        P3D swingTarget = getP3D(swingTrajectories[i].evaluate_linear(remap(cyclePercent, swingStart, swingEnd)));
        ikSolver->addEndEffectorTarget(foot->eeRB, foot->ee->endEffectorOffset, swingTarget);
    }

    void computeStanceTarget(int i) {
        auto foot = robot->getLimb(i);
        stanceTargets[i] = foot->getEEWorldPos();
    }
    void setStanceTarget(int i) {
        auto foot = robot->getLimb(i);
        ikSolver->addEndEffectorTarget(foot->eeRB, foot->ee->endEffectorOffset, stanceTargets[i]);
    }

    double remap(double t, double a, double b) {
        // remap t in [a, b] to [0, 1]
        return (t - a) / (b - a);
    }

    P3D lerp(P3D a, P3D b, double t) {
        return a * (1.0 - t) + b * t;
    }

    double clamp(double t, double a, double b) {
        return std::min(std::max(t, a), b);
    }

    double getCyclePercent(int i, double t) {
        return std::fmod(i * 0.5 * cycleLength + t, cycleLength) / cycleLength;
    }

    bool isStance(double cyclePercent) {
        return stanceStart <= cyclePercent && cyclePercent < stanceEnd;
    }

    bool isSwing(double cyclePercent) {
        return swingStart <= cyclePercent && cyclePercent < swingEnd;
    }

    Phase getPhase(int i, double t) {
        double cyclePercent = getCyclePercent(i, t);
        if (isStance(cyclePercent)) {
            return Phase::Stance;
        } else if (isSwing(cyclePercent)) {
            return Phase::Swing;
        }
    }

    void fixToeAngle(int footIdx) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);
        dVector q;
        gcrr.getQ(q);
        double angle = 0;
        for (int i = 0; i < 3; ++i) {
            angle += q(footJointIndices[footIdx][i] + 6);
        }
        q(6 + footJointIndices[footIdx][3]) = -angle;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
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