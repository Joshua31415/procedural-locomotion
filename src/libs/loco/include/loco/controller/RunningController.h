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
class RunningController : public LocomotionController {
public:
    dVector qDefault;

    std::array<P3D, 2> stanceTargets;
    std::array<P3D, 2> swingTargets;
    std::array<P3D, 2> heelStrikeTargets;
    std::array<Trajectory3D, 2> swingTrajectories;
    std::array<P3D, 2> toeTargets;
    std::array<P3D, 2> heelTargets;
    std::array<P3D, 2> heelStarts;
    std::array<P3D, 2> heelEnds;
    std::array<std::array<double, 2>, 2> toeLimits;
    std::array<bool, 2> heelStartSet;
    std::array<V3D, 2> defaultHeelToToe;
 
    Trajectory1D shoulderJointTrajectory;
    Trajectory1D elbowJointOffsetTrajectory;
    Trajectory1D shoulderTorsionOffsetTrajectory;
    std::array<double, 2> defaultShoulderTorsions;

    int its = 0;
    Trajectory1D verticalOffsetTraj;
public:
    /**
     * constructor
     */
    explicit RunningController(
        const std::shared_ptr<LocomotionTrajectoryPlanner> &planner,
        double cycleLength,
        double stanceStart,
        double swingStart,
        double heelStrikeStart, 
        double shoulderMax = -0.26179938,
        double shoulderMin = 0.5,
        double spineAmplitudeDegree = 7.0,
        double elbowOffsetMaxDegree = -25.0,
        double elbowOffsetMinDegree = 5.0,
        double torsionOffsetAmplitudeDegree = 5.0
    ) 
        : LocomotionController(
            planner,
            cycleLength,
            stanceStart,
            swingStart,
            heelStrikeStart,
            shoulderMax,
            shoulderMin,
            spineAmplitudeDegree
        ) {
        gcrr.getQ(qDefault);
        for (int i : {0, 1}) {
            auto toe = robot->getLimb(i);
            auto heel = robot->getLimb(i + 2);
            defaultHeelToToe[i] = V3D(heel->getEEWorldPos(), toe->getEEWorldPos());
        }

        heelHeight = 0.018125;
        toeHeight = 0.008125;

        for (int i : {0, 1}) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();

            heelStrikeTargets[i] = pToes;
            heelTargets[i] = pHeel;
            heelStarts[i] = pHeel;
            heelEnds[i] = pHeel;
            heelStartSet[i] = false;
            toeTargets[i] = pToes;
            swingTrajectories[i].addKnot(0, V3D(pHeel));
            swingTrajectories[i].addKnot(1, V3D(pHeel));
        }

        for (int i : {0, 1}) {
            auto toeJoint = robot->getJoint(footJointIndices[i][3]);
            toeLimits[i][0] = toeJoint->minAngle;
            toeLimits[i][1] = toeJoint->maxAngle;
        }

        defaultShoulderTorsions[0] = robot->getJoint(armJointIndices[0][2])->defaultJointAngle;
        defaultShoulderTorsions[1] = robot->getJoint(armJointIndices[1][2])->defaultJointAngle; 
            
        verticalOffsetTraj.addKnot(0.0, -0.03);
        verticalOffsetTraj.addKnot(0.15, -0.06);
        verticalOffsetTraj.addKnot(0.4, 0.02);
        verticalOffsetTraj.addKnot(0.65, -0.06);
        verticalOffsetTraj.addKnot(0.9, 0.02);
        verticalOffsetTraj.addKnot(1.0, -0.03);

        shoulderJointTrajectory.addKnot(stanceStart, shoulderMax);
        shoulderJointTrajectory.addKnot(swingStart, shoulderMin);
        shoulderJointTrajectory.addKnot(heelStrikeStart, shoulderMax);

        elbowJointOffsetTrajectory.addKnot(stanceStart, toRad(elbowOffsetMaxDegree));
        elbowJointOffsetTrajectory.addKnot(swingStart, toRad(elbowOffsetMinDegree));
        elbowJointOffsetTrajectory.addKnot(heelStrikeStart, toRad(elbowOffsetMaxDegree));

        shoulderTorsionOffsetTrajectory.addKnot(stanceStart, toRad(torsionOffsetAmplitudeDegree));
        shoulderTorsionOffsetTrajectory.addKnot(swingStart, toRad(0.0));
        shoulderTorsionOffsetTrajectory.addKnot(heelStrikeStart, toRad(torsionOffsetAmplitudeDegree));
    }

    /**
     * destructor
     */
    ~RunningController() override = default;

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        // can maybe just use the walking controller with more/different control points during swing phase?
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        double verticalOffset = verticalOffsetTraj.evaluate_catmull_rom(getCyclePercent(0, t + dt));
        targetPos[1] = planner->trunkHeight + verticalOffset * 0.9;
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
            double cyclePercent = getCyclePercent(i, t);
            double toeAngle;
            double minAngle;
            double maxAngle;
            double h;
            switch (currentPhase) {
                break;
                case Phase::Stance:
                    if (isEarlyStance(getCyclePercent(i, t))) {
                        setHeelToFloor(i);
                        setHeelTarget(i, heelTargets[i]);
                    } 
                    setToeTarget(i, toeTargets[i]);
                    toeAngle = getToeTargetAngle(i);
                    h = std::clamp(remap(cyclePercent, 0.9 * swingStart, swingStart), 0.0, 1.0);
                    minAngle = toeAngle - 1e-6 + lerp(0.0, toeLimits[i][0] - toeAngle + 1e-6, h);
                    maxAngle = toeAngle + 1e-6 + lerp(0.0, toeLimits[i][1] - toeAngle - 1e-6, h);
                    updateToeLimits(i, minAngle, maxAngle); 
                    break;
                case Phase::Swing:
                    // moveToesBackToDefault(i);
                    setToesDefault(i);
                    setHeelTarget(i, heelTargets[i]);
                    // setDefault(i);
                    if (isLateSwing(getCyclePercent(i, t))) {
                        // raiseFootDuringSwing(i);
                    }
                    break;
                case Phase::HeelStrike:
                    setToeTarget(i, toeTargets[i]);
                    setHeelTarget(i, heelTargets[i]);
            }
            setArmAngles(i);
        }
        setSpineAngle();
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        /*
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::Stance) {
                // makeToesParallelToGround(i);
            }
            if (isLateSwing(getCyclePercent(i, t))) {
                // raiseFootDuringSwing(i);
            }
        }
        */
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::HeelStrike) {
                setAnkleAngleDuringHeelStrike(i);
                setToesDefault(i);
            }
        }

        // preparation for the next phase depending on the transition
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            Phase nextPhase = getPhase(i, t + dt);
            if (currentPhase == Phase::Stance && nextPhase == Phase::Swing) {
                // reset toe constraints
                updateToeLimits(i, toeLimits[i][0], toeLimits[i][1]);
                initializeSwingPhase(i);
            }
            if (nextPhase == Phase::Swing) {

                heelTargets[i] = computeHeelTargetSwing(i);
            } else {
                toeTargets[i] = computeToeTarget(i, nextPhase);
                heelTargets[i] = computeHeelTarget(i, nextPhase);
            }
            if (nextPhase == Phase::Stance && isEarlyStance(getCyclePercent(i, t + dt)))
                computeEarlyStanceHeelStrike(i);
        }

        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void updateToeLimits(int footIdx, double lower, double upper) {
        ikSolver->jointLimits(footJointIndices[footIdx][3], 0) = lower;
        ikSolver->jointLimits(footJointIndices[footIdx][3], 1) = upper;
    }

    bool isLateSwing(double cyclePercent) {
        return 0.5 * (swingStart + heelStrikeStart) <= cyclePercent && cyclePercent <= heelStrikeStart;
    }

    void raiseFootDuringSwing(int footIdx) {
        dVector q;
        gcrr.getQ(q);

        double angle = 0;
        for (int i = 0; i < 2; ++i) {
            angle += q(footJointIndices[footIdx][i] + 6);
        }
        double targetAngle = -angle - toRad(15.0);
        double currentAngle = q(footJointIndices[footIdx][2] + 6);
        double cyclePercent = getCyclePercent(footIdx, t);
        q(footJointIndices[footIdx][2] + 6) = lerp(currentAngle, targetAngle, remap(cyclePercent, 0.5 * (swingStart + heelStrikeStart), heelStrikeStart));
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void setDefault(int i) {
        auto heel = robot->getLimb(i + 2);
        heelEnds[i] = (
            heel->limbRoot->getWorldCoordinates() + robot->getHeading() * heel->defaultEEOffsetWorld  // at it's default position
        );
        setHeelTarget(i, heelEnds[i]);
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
            + velocity * cycleLength * (heelStrikeStart - swingStart)  // where the base will be at the end of the swing phase
            + velocityDir * 0.15 // strike roughly under the COM
        );
        // heelEnds[i].z += 0.1; // just for testing while bob is standing
        
        // heelEnds[i][1] = 0.0; // magic number for running...

        V3D p0 = V3D(heelStarts[i]);
        p0 += velocityDir * 0.05;
        p0[1] += 0.02;
        V3D p3 = V3D(heelEnds[i]);
        V3D p1 = lerp(p0, p3, 0.35);
        p1[1] += 0.5;

        V3D p02 = lerp(p0, p1, 0.5);
        V3D p2 = lerp(p0, p3, 0.65);
        // p02 += velocityDir * 0.05;
        p02[1] += 0.05;
        p2[1] += 0.1;
        p2 += velocityDir * 0.3;
        // p2[2] += 0.3; // just for testing while bob is standing
        swingTrajectories[i].clear();
        swingTrajectories[i].addKnot(0.0, p0);
        swingTrajectories[i].addKnot(0.5 * 0.35, p02);
        swingTrajectories[i].addKnot(0.35, p1);
        swingTrajectories[i].addKnot(0.65, p2);
        swingTrajectories[i].addKnot(1.0, p3);
    }

    P3D computeHeelTargetSwing(int i) {
        double cyclePercent = getCyclePercent(i, t);
        // return lerp(heelStarts[i], heelEnds[i], remap(cyclePercent, swingStart, heelStrikeStart));
        return getP3D(swingTrajectories[i].evaluate_catmull_rom(remap(cyclePercent, swingStart, heelStrikeStart)));
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
            P3D pFinal = heel->getEEWorldPos() + defaultHeelToToe[i];
            // pFinal[1] = toeHeight;
            double cyclePercent = getCyclePercent(i, t);
            return lerp(pInitial, pFinal, remap(cyclePercent, heelStrikeStart, 1.0));
        }
        return P3D(0, 0, 0);
    }

    P3D computeHeelTarget(int i, Phase nextPhase) {
        auto heel = robot->getLimb(i + 2);
        if (nextPhase == Phase::HeelStrike) {
            P3D pHeel = heel->getEEWorldPos();
            pHeel[1] = heelHeight;
            heelStartSet[i] = false;
            return pHeel;
        }
        // result isnt actually used for any of the other phases
        return P3D(0, 0, 0);
    }

    void makeFootParallelToGround(int footIdx) {
        dVector q;
        gcrr.getQ(q);
        double angle = 0.0;
        for (int i = 0; i < 2; ++i) {
            angle += q(6 + footJointIndices[footIdx][i]);
        }
        q(6 + footJointIndices[footIdx][2]) = -angle;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    double getToeTargetAngle(int footIdx) {
        dVector q;
        gcrr.getQ(q);
        double angle = 0;
        for (int i = 0; i < 3; ++i) {
            angle += q(footJointIndices[footIdx][i] + 6);
        }
        return -angle;
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
        q(6 + footJointIndices[footIdx][3]) = lerp(
            angleCurrent, angleTarget, 
            std::clamp(
                remap(
                    cyclePercent,
                    swingStart,
                    swingStart + 0.15 * (heelStrikeStart - swingStart)
                ), 0.0, 1.0)
        );
        // q(6 + footJointIndices[footIdx][3]) = 0.0;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();        
    }

    void setToesDefault(int footIdx) {
        dVector q;
        gcrr.getQ(q);
        q(6 + footJointIndices[footIdx][3]) = 0.0;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();        
             
    }

    void setArmAngles(int armIdx) {
        dVector q;
        gcrr.getQ(q);
        // need to map armIdx 1 to 0 and vice-versa, or maybe not?
        int syncedLegIdx;
        if (armIdx == 0) {
            syncedLegIdx = 1;
        } else {
            syncedLegIdx = 0;
        }
        double cyclePercent = getCyclePercent(syncedLegIdx, t);
        double phase = getPhase(syncedLegIdx, t);

        double shoulderTarget = shoulderJointTrajectory.evaluate_catmull_rom(cyclePercent);
        q(6 + armJointIndices[armIdx][0]) = shoulderTarget;

        double elbowOffset = elbowJointOffsetTrajectory.evaluate_catmull_rom(cyclePercent);
        q(6 + armJointIndices[armIdx][1]) = -0.5 * pi + elbowOffset;

        double torsionOffset = shoulderTorsionOffsetTrajectory.evaluate_catmull_rom(cyclePercent);
        // torsionOffset *= -1.0 ? armIdx == 1 : 1.0;
        if (armIdx == 1) {
            torsionOffset *= -1.0;
        }
        q(6 + armJointIndices[armIdx][2]) = defaultShoulderTorsions[armIdx] + torsionOffset;
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

        
        toeTargets[i] = heelTargets[i] + defaultHeelToToe[i];
        toeTargets[i][1] = toeHeight;
    }

    void setHeelStrikeTarget(int i) {
        auto toes = robot->getLimb(i);
        double cyclePercent = getCyclePercent(i, t);
        // maybe fix the starting position?
        P3D pTarget = lerp(toes->getEEWorldPos(), heelStrikeTargets[i], remap(cyclePercent, heelStrikeStart, 1.0));
        ikSolver->addEndEffectorTarget(toes->eeRB, toes->ee->endEffectorOffset, pTarget);
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

        drawEnvMap(*shader);

        /*
        for (auto leg : {0, 1}) {
            drawSphere(toeTargets[leg], 0.02, *shader, {1, 0, 0});
            drawSphere(heelTargets[leg], 0.02, *shader, {1, 0, 0});
        }
        */
        //        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco