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
class RunningController : public LocomotionController {
public:

    Trajectory1D elbowJointOffsetTrajectory{};


    std::array<std::array<double, 2>, 2> toeLimits{};
    std::array<V3D, 2> defaultHeelToToe{};
 
    Trajectory1D shoulderTorsionOffsetTrajectory{};
    std::array<double, 2> defaultShoulderTorsions{};

    Trajectory1D verticalOffsetTraj{};

public:
    /**
     * constructor
     */
    explicit RunningController(
        const std::shared_ptr<LocomotionTrajectoryPlanner> &planner,
        double &cycleLength,
        double &stanceStart,
        double &swingStart,
        double &heelStrikeStart,
        double shoulderMax = -0.26179938,
        double shoulderMin = 0.5,
        double spineAmplitudeDegree = 7.0,
        double pelvisAmplitudeDegree_x = 7.0,
        double pelvisAmplitudeDegree_z = 7.0,
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
            spineAmplitudeDegree,
            pelvisAmplitudeDegree_x,
            pelvisAmplitudeDegree_z
        ) {
        //gcrr.syncGeneralizedCoordinatesWithRobotState();
        
        //P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime());
        //double height;
        //std::cin >> height;
        //targetPos[1] = height;
        //Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime());
        //robot->setRootState(targetPos /* + V3D(0, gui::SimpleGroundModel::getHeight(targetPos), 0)*/, targetOrientation);
        //gcrr.syncGeneralizedCoordinatesWithRobotState();

        for (int i : {0, 1}) {
            auto toe = robot->getLimb(i);
            auto heel = robot->getLimb(i + 2);
            defaultHeelToToe[i] = V3D(heel->getEEWorldPos(), toe->getEEWorldPos());
        }
        heelHeight = 0.008125;
        toeHeight = -0.001875;

        for (int i : {0, 1}) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();

            heelTargets[i] = pHeel;
            heelStarts[i] = pHeel;
            heelEnds[i] = pHeel;
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

    void computeAndApplyControlSignals(double dt) override {

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
                    setToesDefault(i);
                    setHeelTarget(i, heelTargets[i]);
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
                heelTargets[i] = computeHeelTargetSwing(i, dt);
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


    void initializeSwingPhase(int i) {
        V3D velocity = planner->getTargetTrunkVelocityAtTime(planner->getSimTime() /* + dt*/);
        V3D velocityDir = velocity.normalized();
        auto heel = robot->getLimb(i + 2);

        heelStarts[i] = heel->getEEWorldPos();

        heelEnds[i] = (
            heel->limbRoot->getWorldCoordinates() + robot->getHeading() * heel->defaultEEOffsetWorld  // at it's default position
            + velocity * cycleLength * (heelStrikeStart - swingStart)  // where the base will be at the end of the swing phase
            + velocityDir * 0.15 // strike roughly under the COM
        );

        auto tOffset = (swingStart - stanceStart)*cycleLength;

        Quaternion currentOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime());
        Quaternion futureOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + tOffset);

        V3D heelDiff = currentOrientation.inverse() * V3D(heelEnds[i] - heelStarts[i]);


        V3D p0 = V3D(heelStarts[i]);
        p0 += velocityDir * 0.05;
        p0[1] += 0.02;

        V3D p1 = p0 + currentOrientation * heelDiff * 0.35;
        V3D p2 = p1 + currentOrientation.slerp(0.5, futureOrientation) * heelDiff * 0.3;
        V3D p3 = p2 + futureOrientation * heelDiff * 0.35;

        p1[1] += 0.5;
        V3D p02 = p0 + (p1 - p0) * 0.5;

        p02[1] += 0.05;
        p2[1] += 0.1;
        p2 += velocityDir * 0.3;

        heelEnds[i] = getP3D(p3);

        swingTrajectories[i].clear();
        swingTrajectories[i].addKnot(0.0, p0);
        swingTrajectories[i].addKnot(0.5 * 0.35, p02);
        swingTrajectories[i].addKnot(0.35, p1);
        swingTrajectories[i].addKnot(0.65, p2);
        swingTrajectories[i].addKnot(1.0, p3);
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
            P3D pInitial = toes->getEEWorldPos();
            P3D pFinal = heel->getEEWorldPos() + getP3D(robot->getHeading() * V3D(robot->getForward() * heelToeDistance));
            pFinal[1] = toeHeight;

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
            return pHeel;
        }
        // result isnt actually used for any of the other phases
        return P3D(0, 0, 0);
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
        int syncedLegIdx = (armIdx == 0) ? 1 : 0;

        double cyclePercent = getCyclePercent(syncedLegIdx, t);

        double shoulderTarget = shoulderJointTrajectory.evaluate_catmull_rom(cyclePercent);
        q(6 + armJointIndices[armIdx][0]) = shoulderTarget;

        double elbowOffset = elbowJointOffsetTrajectory.evaluate_catmull_rom(cyclePercent);
        q(6 + armJointIndices[armIdx][1]) = -0.5 * pi + elbowOffset;

        double torsionOffset = shoulderTorsionOffsetTrajectory.evaluate_catmull_rom(cyclePercent);

        if (armIdx == 1) {
            torsionOffset *= -1.0;
        }
        q(6 + armJointIndices[armIdx][2]) = defaultShoulderTorsions[armIdx] + torsionOffset;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }


    void computeEarlyStanceHeelStrike(int i) {
        auto toes = robot->getLimb(i);
        auto heel = robot->getLimb(i + 2);
        heelTargets[i] = heel->getEEWorldPos();
        heelTargets[i][1] = heelHeight;

        toeTargets[i][1] = toeHeight;
    }

    void setHeelToFloor(int i) {
        heelTargets[i][1] = heelHeight;
    }

    void drawDebugInfo(gui::Shader *shader) override {

        constexpr size_t numTrajectorySamples = 1000;

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
        /*
        for (auto leg : {0, 1}) {
            drawSphere(toeTargets[leg], 0.02, *shader, {1, 0, 0});
            drawSphere(heelTargets[leg], 0.02, *shader, {1, 0, 0});
        }
        */
        //        planner->drawTrajectories(shader);

        drawEnvMap(*shader);

        if(!gui::SimpleGroundModel::isFlat)
            gui::SimpleGroundModel::groundUneven.draw(*shader, V3D(0, 0, 0));
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco
