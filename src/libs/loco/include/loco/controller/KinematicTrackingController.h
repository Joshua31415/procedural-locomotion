#pragma once

#include <loco/controller/LocomotionController.h>
#include <loco/kinematics/IK_Solver.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>
#include <crl-basic/gui/renderer.h>

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
    double cycleLength = 1.25; // in seconds
    double stanceStart = 0.0; // in percent
    double swingStart = 0.5; // in percent
    double heelStrikeStart = 0.90; // in percent

    std::array<P3D, 2> stanceTargets;
    std::array<P3D, 2> swingTargets;
    std::array<P3D, 2> heelStrikeTargets;
    std::array<Trajectory3D, 2> swingTrajectories;
    std::array<std::array<int, 4>, 2> footJointIndices;
    std::array<P3D, 2> toeTargets;
    std::array<P3D, 2> heelTargets;
    std::array<P3D, 2> heelStarts;
    std::array<P3D, 2> heelEnds;
    std::array<double, 2> heelHeight;
    std::array<bool, 2> heelStartSet;
    std::array<V3D, 2> defaultHeelToToe;
    enum Phase {Stance, Swing, HeelStrike};
    int its = 0;

public:
    /**
     * constructor
     */
    KinematicTrackingController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);

        for (int i = 0; i < 2; ++i) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();
            stanceTargets[i] = pToes;
            defaultHeelToToe[i] = V3D(pHeel, pToes);
            heelStrikeTargets[i] = pToes;
            heelTargets[i] = pHeel;
            heelStarts[i] = pHeel;
            heelEnds[i] = pHeel;
            heelStartSet[i] = false;
            toeTargets[i] = pToes;
            heelHeight[i] = pHeel.y;
            swingTrajectories[i].addKnot(0, V3D(pHeel));
            swingTrajectories[i].addKnot(1, V3D(pHeel));
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
        // before the phase
        for (int i = 0; i < 2; ++i) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::Stance) {
                makeToesParallelToGround(i);
                setToeTarget(i, toeTargets[i]);
            } else if (currentPhase == Phase::Swing) {
                moveToesBackToDefault(i);
                setHeelTarget(i, heelTargets[i]);
            } else if (currentPhase == Phase::HeelStrike) {
                // setToeTarget(i, toeTargets[i]);
                setHeelTarget(i, heelTargets[i]);
            }
        }
        ikSolver->solve();
        for (int i = 0; i < 2; ++i) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::HeelStrike) {
                setAnkleAngleDuringHeelStrike(i);
            }
        }

        // preparation for the next phase depending on the transition
        for (int i = 0; i < 2; ++i) {
            Phase currentPhase = getPhase(i, t);
            Phase nextPhase = getPhase(i, t + dt);
            if (currentPhase == Phase::Stance && nextPhase == Phase::Swing) {
                initializeSwingPhase(i);
            }
            if (nextPhase == Phase::Swing) {
                heelTargets[i] = computeHeelTargetSwing(i);
            } else {
                toeTargets[i] = computeToeTarget(i, nextPhase);
                heelTargets[i] = computeHeelTarget(i, nextPhase);    
            }
            /*
            toeTargets[i] = computeToeTarget(i, nextPhase);
            heelTargets[i] = computeHeelTarget(i, nextPhase);
            */
        }



    }

    void initializeSwingPhase(int i) {
        V3D velocity = planner->getTargetTrunkVelocityAtTime(planner->getSimTime() /* + dt*/);
        V3D velocityDir = velocity.normalized();
        auto heel = robot->getLimb(i + 2);

        double stepLength = 0.1;
        heelStarts[i] = heel->getEEWorldPos();
        // heelEnds[i] = heelStarts[i];
        // heelEnds[i].y = heelHeight[i];
        // heelEnds[i] = heelEnds[i] + velocityDir * stepLength;
        heelEnds[i] = (
            heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld  // at it's default position
            + velocity * cycleLength * (heelStrikeStart - swingStart)       // where the default will be at the end of the swing phase
            + (robot->getHeading() * robot->getForward()) * stepLength
        );

        V3D p0 = V3D(heelStarts[i]);
        V3D p3 = V3D(heelEnds[i]);
        V3D p1 = lerp(p0, p3, 0.25);
        V3D p2 = lerp(p1, p3, 0.75);
        p1[1] += 0.1;
        p2[1] += 0.05;

        swingTrajectories[i].clear();
        swingTrajectories[i].addKnot(0.0, p0);
        swingTrajectories[i].addKnot(0.25, p1);
        swingTrajectories[i].addKnot(0.75, p2);
        swingTrajectories[i].addKnot(1.0, p3);
    }

    P3D computeHeelTargetSwing(int i) {
        double cyclePercent = getCyclePercent(i, t);
        // return lerp(heelStarts[i], heelEnds[i], remap(cyclePercent, swingStart, heelStrikeStart));
        return getP3D(swingTrajectories[i].evaluate_catmull_rom(remap(cyclePercent, swingStart, heelStrikeStart)));
    }

    void setHeelTarget(int i, P3D pTarget) {
        auto heel = robot->getLimb(i + 2);
        ikSolver->addEndEffectorTarget(heel->eeRB, heel->ee->endEffectorOffset, pTarget);
    }
    
    void setToeTarget(int i, P3D pTarget) {
        auto foot = robot->getLimb(i);
        ikSolver->addEndEffectorTarget(foot->eeRB, foot->ee->endEffectorOffset, pTarget); 
    }

    P3D computeToeTarget(int i, Phase nextPhase) {
        auto toes = robot->getLimb(i);
        auto heel = robot->getLimb(i + 2);

        if (nextPhase == Phase::Stance) {
            return toes->getEEWorldPos();
        } else if (nextPhase == Phase::Swing) {
            return P3D(0, 0, 0);
        } else if (nextPhase == Phase::HeelStrike) {
            // might be easier to just model this via a spline for the ankle angle
            P3D pInitial = toes->getEEWorldPos();
            P3D pFinal = heel->getEEWorldPos() + defaultHeelToToe[i];
            double cyclePercent = getCyclePercent(i, t);
            return lerp(pInitial, pFinal, remap(cyclePercent, heelStrikeStart, 1.0));
        }
    }

    P3D computeHeelTarget(int i, Phase nextPhase) {
        auto heel = robot->getLimb(i + 2);
        if (nextPhase == Phase::Stance) {
            return P3D(0, 0, 0);
        } else if (nextPhase == Phase::Swing) {
            if (!heelStartSet[i]) {
                // not sure if this should interpolate between the current position and the target 
                // or the start position of the swing phase and the target
                P3D pStart = heel->getEEWorldPos();
                heelStarts[i] = pStart;
                heelStartSet[i] = true; 
            }
            P3D pEnd = (heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld  // heel position in default pose
                        + 0.05 * (robot->getHeading() * robot->getForward())                // offset in heading direction
            );
            return lerp(heelStarts[i], pEnd, remap(getCyclePercent(i, t), swingStart, heelStrikeStart));
        } else if (nextPhase == Phase::HeelStrike) {
            P3D pHeel = heel->getEEWorldPos();
            heelStartSet[i] = false;
            return pHeel;
        }
    }

    
    void setAnkleAngleDuringHeelStrike(int footIdx) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);
        dVector q;
        gcrr.getQ(q);
        double angle = 0;
        for (int i = 0; i < 2; ++i) {
            angle += q(footJointIndices[footIdx][i] + 6);
        }
        double cyclePercent = getCyclePercent(footIdx, t);
        double angleTarget = -angle;
        double angleCurrent = q(6 + footJointIndices[footIdx][2]);

        q(6 + footJointIndices[footIdx][2]) = lerp(angleCurrent, angleTarget, remap(cyclePercent, heelStrikeStart, 1.0));
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void makeToesParallelToGround(int footIdx) {
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

    void moveToesBackToDefault(int footIdx) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);
        dVector q;
        gcrr.getQ(q);
        double angleCurrent = q(6 + footJointIndices[footIdx][3]);
        double angleTarget = 0.0;
        double cyclePercent = getCyclePercent(footIdx, t);
        q(6 + footJointIndices[footIdx][3]) = lerp(angleCurrent, angleTarget, remap(cyclePercent, swingStart, heelStrikeStart));
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();        
    }

    void computeStanceTarget(int i) {
        auto foot = robot->getLimb(i);
        stanceTargets[i] = foot->getEEWorldPos();
    }

    void setStanceTarget(int i) {
        auto foot = robot->getLimb(i);
        ikSolver->addEndEffectorTarget(foot->eeRB, foot->ee->endEffectorOffset, stanceTargets[i]);
    }
    
    void computeSwingTrajectory(int i) {
        auto foot = robot->getLimb(i + 2);

        swingTrajectories[i].clear();
        P3D pStart = foot->getEEWorldPos();
        V3D offset(pStart, foot->limbRoot->getWorldCoordinates() + foot->defaultEEOffsetWorld);
        P3D pEnd = pStart + offset * 2.1;
        swingTrajectories[i].addKnot(0, V3D(pStart));
        swingTrajectories[i].addKnot(1, V3D(pEnd));
    }

    void computeSwingTarget(int i) {
        auto heel = robot->getLimb(i + 2);
        P3D pStart = heel->getEEWorldPos();
        P3D pEnd = (heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld  // heel position in default pose
                    + 0.05 * (robot->getHeading() * robot->getForward())                // offset in heading direction
        );

        swingTargets[i] = lerp(pStart, pEnd, remap(getCyclePercent(i, t), swingStart, heelStrikeStart));
    }

    void lockSwingTargetForHeelStrike(int i) {
        auto heel = robot->getLimb(i + 2);
        P3D pHeel = heel->getEEWorldPos();
        swingTargets[i] = pHeel;
    }

    void setSwingTarget(int i) {
        auto heel = robot->getLimb(i + 2);
        ikSolver->addEndEffectorTarget(heel->eeRB, heel->ee->endEffectorOffset, swingTargets[i]);
    }

    void computeHeelStrikeTarget(int i) {
        auto heel = robot->getLimb(i + 2);
        heelStrikeTargets[i] = heel->getEEWorldPos() + defaultHeelToToe[i];
    }

    void setHeelStrikeTarget(int i) {
        auto toes = robot->getLimb(i);
        double cyclePercent = getCyclePercent(i, t);
        // maybe fix the starting position?
        P3D pTarget = lerp(toes->getEEWorldPos(), heelStrikeTargets[i], remap(cyclePercent, heelStrikeStart, 1.0));
        ikSolver->addEndEffectorTarget(toes->eeRB, toes->ee->endEffectorOffset, pTarget);
    }

    double remap(double t, double a, double b) {
        // remap t in [a, b] to [0, 1]
        return (t - a) / (b - a);
    }
    
    template <typename T>
    T lerp(T  a, T b, double t) {
        return a * (1.0 - t) + b * t;
    }

    double clamp(double t, double a, double b) {
        return std::min(std::max(t, a), b);
    }

    double getCyclePercent(int i, double t) {
        return std::fmod(i * 0.5 * cycleLength + t, cycleLength) / cycleLength;
    }

    bool isStance(double cyclePercent) {
        return stanceStart <= cyclePercent && cyclePercent < swingStart;
    }

    bool isSwing(double cyclePercent) {
        return swingStart <= cyclePercent && cyclePercent < heelStrikeStart;
    }
    
    bool isHeelStrike(double cyclePercent) {
        return heelStrikeStart <= cyclePercent && cyclePercent < 1.0;
    }

    Phase getPhase(int i, double t) {
        double cyclePercent = getCyclePercent(i, t);
        if (isStance(cyclePercent)) {
            return Phase::Stance;
        } else if (isSwing(cyclePercent)) {
            return Phase::Swing;
        } else if (isHeelStrike(cyclePercent)) {
            return Phase::HeelStrike;
        }
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