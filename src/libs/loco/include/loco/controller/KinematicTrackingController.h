#pragma once

#include <loco/controller/LocomotionController.h>
#include <loco/kinematics/IK_Solver.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>
#include <crl-basic/gui/renderer.h>

namespace crl::loco {


/* TODO
 * Smooth transition between Phases
 * walking backwards
 * walking in a curve
*/


/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    std::shared_ptr<LeggedRobot> robot;
    std::shared_ptr<IK_Solver> ikSolver = nullptr;
    GeneralizedCoordinatesRobotRepresentation gcrr;

    double heelHeight;
    double toeHeight;


    double t = 0;
    double cycleLength = 1.25; // in seconds

    double stanceStart = 0.0; // in percent
    double swingStart = 0.6; // in percent
    double heelStrikeStart = 0.9; // in percent

    double heelToeDistance;

    std::array<P3D, 2> stanceTargets;
    std::array<P3D, 2> swingTargets;
    std::array<P3D, 2> heelStrikeTargets;
    std::array<Trajectory3D, 2> swingTrajectories;
    std::array<std::array<int, 4>, 2> footJointIndices;
    std::array<P3D, 2> toeTargets;
    std::array<P3D, 2> heelTargets;
    std::array<P3D, 2> heelStarts;
    std::array<P3D, 2> heelEnds;
//    std::array<double, 2> heelHeight;
    std::array<bool, 2> heelStartSet;
    enum Phase {Stance, Swing, HeelStrike};
    int its = 0;

public:
    /**
     * constructor
     */
    explicit KinematicTrackingController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner), gcrr(planner->robot) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);

        auto toe = robot->getLimb(0);
        auto heel = robot->getLimb(2);

        heelToeDistance = V3D(heel->getEEWorldPos(), toe->getEEWorldPos()).norm();

        heelHeight = (heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld)[1];
        toeHeight = (toe->limbRoot->getWorldCoordinates() + toe->defaultEEOffsetWorld)[1];

        for (int i : {0, 1}) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();
            stanceTargets[i] = pToes;
            heelStrikeTargets[i] = pToes;
            heelTargets[i] = pHeel;
            heelStarts[i] = pHeel;
            heelEnds[i] = pHeel;
            heelStartSet[i] = false;
            toeTargets[i] = pToes;
//            heelHeight[i] = pHeel.y;
            swingTrajectories[i].addKnot(0, V3D(pHeel));
            swingTrajectories[i].addKnot(1, V3D(pHeel));
        }

        std::array lJoints = {"lHip_1", "lKnee", "lAnkle_1", "lToeJoint"};
        std::array rJoints = {"rHip_1", "rKnee", "rAnkle_1", "rToeJoint"};
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
    ~KinematicTrackingController() override = default;

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }


    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.

        gcrr.syncGeneralizedCoordinatesWithRobotState();
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
//        targetPos[1] = 0.88;
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);
        robot->setRootState(targetPos, targetOrientation);

        
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        t += dt;


        // use the toe as the targeted endeffector in the stance phase
        // use the heel as the targeted endeffector in the swing phase
        // idx i == 0 <-> right leg
        // before the phase
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);

            switch(currentPhase) {
            break; case Phase::Stance:
                makeToesParallelToGround(i);
                setToesToFloor(i);
                if(isEarlyStance(getCyclePercent(i, t))){
                    setHeelToFloor(i);
                    setHeelTarget(i, heelTargets[i]);
                }
                setToeTarget(i, toeTargets[i]);
            break; case Phase::Swing:
                moveToesBackToDefault(i);
                setHeelTarget(i, heelTargets[i]);
            break; case Phase::HeelStrike:
                setToeTarget(i, toeTargets[i]);
                setHeelTarget(i, heelTargets[i]);
            }

        }
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::HeelStrike) {
                setAnkleAngleDuringHeelStrike(i);
            }
        }

        // preparation for the next phase depending on the transition
        for (int i : {0, 1}) {
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
            if(nextPhase == Phase::Stance && isEarlyStance(getCyclePercent(i, t+dt)))
                computeEarlyStanceHeelStrike(i);
        }

        gcrr.syncGeneralizedCoordinatesWithRobotState();
    }

    void addArmTargets(double dt) {
        for (uint i = 4; i < robot->getLimbCount(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(robot->getLimb(i), planner->getSimTime() + dt);
            ikSolver->addEndEffectorTarget(robot->getLimb(i)->eeRB, robot->getLimb(i)->ee->endEffectorOffset, target);
        }
    }

    void initializeSwingPhase(int i) {
        V3D velocity = planner->getTargetTrunkVelocityAtTime(planner->getSimTime() /* + dt*/);
        V3D velocityDir = velocity.normalized();
        auto heel = robot->getLimb(i + 2);

        const double stepLength = velocity.norm() * cycleLength;


        heelStarts[i] = heel->getEEWorldPos();
        // heelEnds[i] = heelStarts[i];
        // heelEnds[i].y = heelHeight[i];
        // heelEnds[i] = heelEnds[i] + velocityDir * stepLength;
        heelEnds[i] = (
            heel->limbRoot->getWorldCoordinates() + robot->getHeading() * heel->defaultEEOffsetWorld  // at it's default position
            + velocity * cycleLength * (heelStrikeStart - swingStart)       // where the default will be at the end of the swing phase
            //TODO add rotational velocity
            );

        if((robot->getHeading() * robot->getForward()).dot(velocity) >= 0){
            heelEnds[i] = heelEnds[i] + robot->getHeading() * velocity.normalized() * stepLength/3; //velocity dependent offset
        }else{
            assert(false && "Walking backwards is not supported yet.");
        }

        heelEnds[i][1] = heelHeight;

        V3D p0 = V3D(heelStarts[i]);
        V3D p3 = V3D(heelEnds[i]);
        V3D p1 = lerp(p0, p3, 0.25);
        V3D p2 = lerp(p1, p3, 0.75);
        p1[1] += 0.07;
        p2[1] += 0.03;

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

        switch(nextPhase){
        break;case Phase::Stance:
            return toes->getEEWorldPos();

        break;case Phase::Swing:
            return P3D(0, 0, 0);

        break;case Phase::HeelStrike:
            // might be easier to just model this via a spline for the ankle angle
            P3D pInitial = toes->getEEWorldPos();
            P3D pFinal = heel->getEEWorldPos() + getP3D(robot->getHeading() * V3D(robot->getForward() * heelToeDistance));
            pFinal[1] = toeHeight;
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
            P3D pEnd = heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld  // heel position in default pose
                       + getP3D(robot->getHeading() * V3D(+ 0.05 * robot->getForward()));  // offset in forward direction


            pEnd[1] = heelHeight;

            return lerp(heelStarts[i], pEnd, remap(getCyclePercent(i, t), swingStart, heelStrikeStart));
        } else if (nextPhase == Phase::HeelStrike) {
            P3D pHeel = heel->getEEWorldPos();
            pHeel[1] = heelHeight;
            heelStartSet[i] = false;
            return pHeel;
        }
    }

    void setAnkleAngleDuringHeelStrike(int footIdx) {
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
        auto heel = robot->getLimb(i + 2);

        swingTrajectories[i].clear();
        P3D pStart = heel->getEEWorldPos();
        V3D offset(pStart, heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld);
        P3D pEnd = pStart + offset * 2.1;
        swingTrajectories[i].addKnot(0, V3D(pStart));
        swingTrajectories[i].addKnot(1, V3D(pEnd));
    }

    void computeSwingTarget(int i) {
        auto heel = robot->getLimb(i + 2);
        P3D pStart = heel->getEEWorldPos();
        P3D pEnd = getP3D(robot->getHeading() * V3D(heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld  // heel position in default pose
                    + 0.05 * robot->getForward()));             // offset in heading direction


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

    void computeEarlyStanceHeelStrike(int i) {
        auto toes = robot->getLimb(i);
        auto heel = robot->getLimb(i + 2);
        heelTargets[i] = heel->getEEWorldPos();
        heelTargets[i][1] = heelHeight;
    }

    void computeHeelStrikeTarget(int i) {
        auto heel = robot->getLimb(i + 2);
        heelStrikeTargets[i] = heel->getEEWorldPos() + robot->getForward() * heelToeDistance;
    }

    void setHeelStrikeTarget(int i) {
        auto toes = robot->getLimb(i);
        double cyclePercent = getCyclePercent(i, t);
        // maybe fix the starting position?
        P3D pTarget = lerp(toes->getEEWorldPos(), heelStrikeTargets[i], remap(cyclePercent, heelStrikeStart, 1.0));
        ikSolver->addEndEffectorTarget(toes->eeRB, toes->ee->endEffectorOffset, pTarget);
    }

    static double remap(double time, double a, double b) {
        // remap t in [a, b] to [0, 1]
        return (time - a) / (b - a);
    }

    template <typename T>
    T lerp(T  a, T b, double time) {
        return a * (1.0 - time) + b * time;
    }

    double getCyclePercent(int i, double time) const {
        return std::fmod(i * 0.5 * cycleLength + time, cycleLength) / cycleLength;
    }

    bool isStance(double cyclePercent) const {
        return stanceStart <= cyclePercent && cyclePercent < swingStart;
    }

    bool isEarlyStance(double cyclePercent) const {
        return isStance(cyclePercent) && (cyclePercent < (stanceStart + swingStart)*0.5);
    }

    bool isSwing(double cyclePercent) const {
        return swingStart <= cyclePercent && cyclePercent < heelStrikeStart;
    }
    
    bool isHeelStrike(double cyclePercent) const {
        return heelStrikeStart <= cyclePercent && cyclePercent < 1.0;
    }

    Phase getPhase(int i, double time) const {
        double cyclePercent = getCyclePercent(i, time);
        if (isStance(cyclePercent)) {
            return Phase::Stance;
        } else if (isSwing(cyclePercent)) {
            return Phase::Swing;
        } else if (isHeelStrike(cyclePercent)) {
            return Phase::HeelStrike;
        }
    }

    void setToesToFloor(int i) {
        toeTargets[i][1] = toeHeight;
    }

    void setHeelToFloor(int i) {
        heelTargets[i][1] = heelHeight;
    }

    void advanceInTime(double dt) override {
        planner->advanceInTime(dt);
    }

    void drawDebugInfo(gui::Shader *shader) override {

        constexpr size_t numTrajectorySamples = 100;


        for(auto leg : {0, 1}){
            switch(getPhase(leg ,t)){
                break;case Stance:
                    drawSphere(toeTargets[leg], 0.02, *shader, {1, 0, 0});
                    if(isEarlyStance(getCyclePercent(leg, t)))
                        drawSphere(heelTargets[leg], 0.02, *shader, {1, 0, 0});
                break;case Swing:
                    for(int i = 0; i < numTrajectorySamples; ++i){
                        drawSphere(getP3D(swingTrajectories[leg].evaluate_catmull_rom(i*1.0/numTrajectorySamples)), 0.002, *shader, {0, 1, 0});
                    }
                    drawSphere(heelEnds[leg], 0.02, *shader, {0, 0, 0});
                break;case HeelStrike:
                    drawSphere(toeTargets[leg], 0.02, *shader, {0, 0, 1});
                    drawSphere(heelTargets[leg], 0.02, *shader, {0, 0, 1});
            }
        }

//        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco