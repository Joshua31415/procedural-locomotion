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

    Trajectory1D elbowJointTrajectory{};

    std::array<P3D, 2> toeStrikeTarget{};

    const double elbowSwingOffset = 0.03;

public:
    /**
     * constructor
     */
    explicit KinematicTrackingController(
        const std::shared_ptr<LocomotionTrajectoryPlanner> &planner,
        double &cycleLength,
        double &stanceStart,
        double &swingStart,
        double &heelStrikeStart,
        double shoulderMax = -0.0872664626, // 5 degree
        double shoulderMin = 0.0872664626,
        double elbowMin = -0.25,
        double elbowMax = -0.45,
        double spineAmplitudeDegree = 2.0,
        double pelvisAmplitudeDegree_x = 2.0,
        double pelvisAmplitudeDegree_z = 2.0
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
            pelvisAmplitudeDegree_z){
        for (int i : {0, 1}) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();
            heelTargets[i] = pHeel;
            heelStarts[i] = pHeel;
            heelEnds[i] = pHeel;
            toeTargets[i] = pToes;
            toeStrikeTarget[i] = pToes;
            swingTrajectories[i].addKnot(0, V3D(pHeel));
            swingTrajectories[i].addKnot(1, V3D(pHeel));
        }

        shoulderJointTrajectory.addKnot(stanceStart, shoulderMax);
        shoulderJointTrajectory.addKnot(swingStart, shoulderMin);
        shoulderJointTrajectory.addKnot(heelStrikeStart, shoulderMax);

        elbowJointTrajectory.addKnot(stanceStart, elbowMax);
        elbowJointTrajectory.addKnot(swingStart + elbowSwingOffset, elbowMin);
        elbowJointTrajectory.addKnot(heelStrikeStart + elbowSwingOffset, elbowMax);
    }

    /**
     * destructor
     */
    ~KinematicTrackingController() override = default;

    void computeAndApplyControlSignals(double dt) override {
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        double cyclePercent = getCyclePercent(0, planner->getSimTime() + dt);
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt, cyclePercent);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);
        robot->setRootState(targetPos + V3D(0, gui::SimpleGroundModel::getHeight(targetPos), 0), targetOrientation);

        gcrr.syncGeneralizedCoordinatesWithRobotState();
        t += dt;

        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);

            switch(currentPhase) {
            break; case Stance:
                makeToesParallelToGround(i);
                setToesToFloor(i, targetPos);
                if(isEarlyStance(cyclePercent)){
                    setHeelToFloor(i, targetPos);
                    //Smoothly reduce weight of target to zero until end of early stance
                    setHeelTarget(i, heelTargets[i], 1 - remap(cyclePercent, stanceStart, swingStart*earlyStanceDuration));
                }
                setToeTarget(i, toeTargets[i]);
            break; case Swing:
                moveToesBackToDefault(i);
                setHeelTarget(i, heelTargets[i]);
            break; case HeelStrike:
                setToeTarget(i, toeTargets[i], 0.1);
                setHeelTarget(i, heelTargets[i]);
            }
            setArmAngles(i); 
        }
        setSpineAngle();
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();

        // preparation for the next phase depending on the transition
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            Phase nextPhase = getPhase(i, t + dt);
            if (currentPhase == Stance && nextPhase == Swing) {
                initializeSwingPhase(i);
            }
            if (nextPhase == Swing) {
                heelTargets[i] = computeHeelTargetSwing(i, dt);
            } else {
                toeTargets[i] = computeToeTarget(i, nextPhase, dt);
                heelTargets[i] = computeHeelTarget(i, nextPhase);
            }
            if(nextPhase == Stance && isEarlyStance(getCyclePercent(i, t+dt)))
                computeEarlyStanceHeelStrike(i);
        }

        setAnkleTiltsToZero();

        gcrr.syncGeneralizedCoordinatesWithRobotState();

    }

    void setAnkleTiltsToZero(){

        dVector q;
        gcrr.getQ(q);

        double lAngle = q(6 + robot->getJointByName("lHip_2")->jIndex);
        double rAngle = q(6 + robot->getJointByName("rHip_2")->jIndex);


        q(6 + robot->getJointByName("rAnkle_2")->jIndex) = -rAngle;//lerp(rAngle, 0.0, remap(rCyclePercent, heelStrikeStart, 1.0));
        q(6 + robot->getJointByName("lAnkle_2")->jIndex) = -lAngle;//lerp(lAngle, 0.0, remap(lCyclePercent, heelStrikeStart, 1.0));


        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();

    }

    void initializeSwingPhase(int i) {
        V3D velocity = planner->getTargetTrunkVelocityAtTime(planner->getSimTime() /* + dt*/);
        V3D velocityDir = velocity.normalized();
        auto heel = robot->getLimb(i + 2);
        auto heading = robot->getHeading();

        const double stepLength = velocity.norm() * cycleLength;

        heelStarts[i] = heel->getEEWorldPos();

        heelEnds[i] = (
            heel->limbRoot->getWorldCoordinates() + heading * heel->defaultEEOffsetWorld  // at it's default position
            + velocity * cycleLength * (heelStrikeStart - swingStart)       // where the default should be at the end of the swing phase
            );

        if((robot->getHeading() * robot->getForward()).dot(velocity) >= 0){
            heelEnds[i] = heelEnds[i] + velocity.normalized() * stepLength*0.3; //velocity dependent offset
        }else{
            assert(false && "Walking backwards is not supported yet.");
        }

        heelEnds[i][1] = heelHeight + gui::SimpleGroundModel::getHeight(heelEnds[i]);


        auto tOffset = (swingStart - stanceStart)*cycleLength;

        Quaternion currentOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime());
        Quaternion futureOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + tOffset);

        //Total displacement of the heel rotated to default pose
        V3D heelDiff = currentOrientation.inverse() * V3D(heelEnds[i] - heelStarts[i]);

        //spline points get computed by piecewise application of rotation during strike phase
        V3D p1 = V3D(heelStarts[i] + currentOrientation * heelDiff * 0.25);
        V3D p2 = p1 + currentOrientation.slerp(0.5, futureOrientation) * heelDiff * 0.5;
        V3D p3 = p2 + futureOrientation * heelDiff * 0.25;

        heelEnds[i] = getP3D(p3);

        V3D p0 = V3D(heelStarts[i]);
        p1[1] += 0.07;
        p2[1] += 0.03;

        swingTrajectories[i].clear();
        swingTrajectories[i].addKnot(0.0, p0);
        swingTrajectories[i].addKnot(0.25, p1);
        swingTrajectories[i].addKnot(0.75, p2);
        swingTrajectories[i].addKnot(1.0, p3);
    }



    [[nodiscard]] P3D computeToeTarget(int i, Phase nextPhase, double dt) {
        auto toes = robot->getLimb(i);
        auto heel = robot->getLimb(i + 2);

        switch(nextPhase){
        break;case Stance:{
            P3D pInitial = toes->getEEWorldPos();
            pInitial[1] = toeHeight + gui::SimpleGroundModel::getHeight(pInitial);
            return pInitial;
        }

        break;case Swing:
            return P3D(0, 0, 0);

        break;case HeelStrike:
            P3D pInitial = toes->getEEWorldPos();
            double cyclePercent = getCyclePercent(i, t+dt);
            return lerp(pInitial, toeStrikeTarget[i], remap(cyclePercent, heelStrikeStart, 1.0)); // TODO make circular trajectory
        }
    }

    [[nodiscard]] P3D computeHeelTarget(int i, Phase nextPhase) {
        auto heel = robot->getLimb(i + 2);
        if (nextPhase == HeelStrike) {
            P3D pHeel = getP3D(swingTrajectories[i].evaluate_catmull_rom(1.0));

            P3D pFinal = pHeel + getP3D(robot->getHeading() * V3D(robot->getForward() * heelToeDistance));
            pFinal[1] = toeHeight + gui::SimpleGroundModel::getHeight(pFinal);

            toeStrikeTarget[i] = pFinal;

            pHeel[1] = heelHeight + gui::SimpleGroundModel::getHeight(pHeel);
            return pHeel;
        }
        return P3D(0, 0, 0);
    }

    void makeToesParallelToGround(int footIdx) {
        dVector q;
        gcrr.getQ(q);
        double angle = 0;
        for (int i : {0, 1, 2}) {
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

    void setArmAngles(int armIdx) {
        dVector q;
        gcrr.getQ(q);

        double speed_direction = planner->speedForward;
        int syncedLegIdx;
        // Need a smooth transition
        if ((armIdx == 0 && speed_direction >= 0) || (armIdx == 1 && speed_direction < 0)) {
            syncedLegIdx = 1;
        } else {
            syncedLegIdx = 0;
        }
        double cyclePercent = getCyclePercent(syncedLegIdx, t);

        // for the first frame
        if (t <= cycleLength){
            double shoulderTarget = shoulderJointTrajectory.evaluate_catmull_rom(cyclePercent);
            q(6 + armJointIndices[armIdx][0]) = lerp(q(6 + armJointIndices[armIdx][0]), shoulderTarget, cyclePercent);
            double elbowTarget = elbowJointTrajectory.evaluate_catmull_rom(cyclePercent);
            q(6 + armJointIndices[armIdx][1]) = lerp(q(6 + armJointIndices[armIdx][1]), elbowTarget, cyclePercent);
        }
        else {
            double shoulderTarget = shoulderJointTrajectory.evaluate_catmull_rom(cyclePercent);
            q(6 + armJointIndices[armIdx][0]) = shoulderTarget;
            double elbowTarget = elbowJointTrajectory.evaluate_catmull_rom(cyclePercent);
            q(6 + armJointIndices[armIdx][1]) = elbowTarget;
        }

        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void computeEarlyStanceHeelStrike(int i) {
        auto toes = robot->getLimb(i);
        auto heel = robot->getLimb(i + 2);
        heelTargets[i] = heel->getEEWorldPos();
        heelTargets[i][1] = heelHeight + gui::SimpleGroundModel::getHeight(heelTargets[i]);
    }

    void setToesToFloor(int i, const P3D& position) {
        toeTargets[i][1] = toeHeight + gui::SimpleGroundModel::getHeight(position);
    }

    void setHeelToFloor(int i, const P3D& position) {
        heelTargets[i][1] = heelHeight + gui::SimpleGroundModel::getHeight(position);
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
                    drawSphere(toeStrikeTarget[leg], 0.02, *shader, {0, 1, 0});
                    drawSphere(toeTargets[leg], 0.02, *shader, {0, 0, 1});
                    drawSphere(heelTargets[leg], 0.02, *shader, {0, 0, 1});
            }
        }

        for(const auto&[p, r, c] : drawList)
            drawSphere(p, r, *shader, c);


        drawList.clear();

//        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco