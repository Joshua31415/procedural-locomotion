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
    const double elbowMin;
    const double elbowMax;
    const double elbowSwingOffset = 0.03;

    //Vector of point/radius/color
    std::vector<std::tuple<P3D, double, V3D>> drawList{};

    std::array<P3D, 2> stanceTargets;
    std::array<P3D, 2> swingTargets;
    std::array<P3D, 2> heelStrikeTargets;
    std::array<Trajectory3D, 2> swingTrajectories;

    std::array<P3D, 2> toeTargets;
    std::array<P3D, 2> toeStrikeTarget;
    std::array<P3D, 2> heelTargets;
    std::array<P3D, 2> heelStarts;
    std::array<P3D, 2> heelEnds;

    std::array<bool, 2> heelStartSet;

    Trajectory1D shoulderJointTrajectory;
    Trajectory1D elbowJointTrajectory;

    int its = 0;

public:
    /**
     * constructor
     */
    explicit KinematicTrackingController(
        const std::shared_ptr<LocomotionTrajectoryPlanner> &planner,
        double cycleLength,
        double stanceStart,
        double swingStart,
        double heelStrikeStart,
        double shoulderMax = -0.0872664626, // 5 degree
        double shoulderMin = 0.0872664626,
        double elbowMin = -0.25,
        double elbowMax = -0.45,
        double spineAmplitudeDegree = 2.0
    ) 
        : LocomotionController(
            planner,
            cycleLength,
            stanceStart,
            swingStart,
            heelStrikeStart,
            shoulderMax,
            shoulderMin,
            spineAmplitudeDegree),
          elbowMin(elbowMin),
          elbowMax(elbowMax) {
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
            toeStrikeTarget[i] = pToes;
//            heelHeight[i] = pHeel.y;
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

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }


    void computeAndApplyControlSignals(double deltaT) override {
        double dt = deltaT;
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);
        robot->setRootState(targetPos + V3D(0, gui::SimpleGroundModel::getHeight(targetPos), 0), targetOrientation);

        gcrr.syncGeneralizedCoordinatesWithRobotState();
        t += dt;

        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);

            switch(currentPhase) {
            break; case Phase::Stance:
                makeToesParallelToGround(i);
                setToesToFloor(i, targetPos);
                if(isEarlyStance(getCyclePercent(i, t))){
                    setHeelToFloor(i, targetPos);
                    setHeelTarget(i, heelTargets[i]);
                }
                setToeTarget(i, toeTargets[i]);
            break; case Phase::Swing:
                setHipAngleToTangent(i);
                moveToesBackToDefault(i);
                setHeelTarget(i, heelTargets[i]);
            break; case Phase::HeelStrike:
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
            if(nextPhase == Phase::Stance && isEarlyStance(getCyclePercent(i, t+dt)))
                computeEarlyStanceHeelStrike(i);
        }

        setAnklesToZero();

        gcrr.syncGeneralizedCoordinatesWithRobotState();
    }

    void setHipAngleToTangent(int legIdx){
        const auto hipJointIndex = (legIdx == 0) ?
                robot->getJointByName("rHip_torsion")->jIndex
            :   robot->getJointByName("lHip_torsion")->jIndex;

//        const double dtSpline = remap(swingStart + dt, swingStart, heelStrikeStart);
//
//        const double tSpline = remap(getCyclePercent(legIdx, t), swingStart, heelStrikeStart);
//
//
//        const auto tangent = (swingTrajectories[legIdx].evaluate_catmull_rom(tSpline+dtSpline)
//                             - swingTrajectories[legIdx].evaluate_catmull_rom(tSpline))
//                                 .normalized();

        dVector q;
        gcrr.getQ(q);

        double cyclePercent = getCyclePercent(legIdx, t);

        q(6 + hipJointIndex) = lerp(q(6 + hipJointIndex), 0.0, remap(cyclePercent, swingStart, heelStrikeStart));


        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void setAnklesToZero(){

        dVector q;
        gcrr.getQ(q);


        double rCyclePercent = getCyclePercent(0, t);
        double lCyclePercent = getCyclePercent(1, t);

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
            heelEnds[i] = heelEnds[i] + velocity.normalized() * stepLength/3; //velocity dependent offset
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
        V3D p2 = p1 + currentOrientation.slerp(0.25, futureOrientation) * heelDiff * 0.5;
        V3D p3 = p2 + currentOrientation.slerp(0.75, futureOrientation) * heelDiff * 0.25;

        heelEnds[i] = getP3D(p3);

        V3D p0 = V3D(heelStarts[i]);
//        V3D p3 = V3D(heelEnds[i]);
//        V3D p1 = lerp(p0, p3, 0.25);
//        V3D p2 = lerp(p1, p3, 0.75);
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
            double cyclePercent = getCyclePercent(i, t);
            return lerp(pInitial, toeStrikeTarget[i], remap(cyclePercent, heelStrikeStart, 1.0));
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


            pEnd[1] = heelHeight + gui::SimpleGroundModel::getHeight(pEnd);

            return lerp(heelStarts[i], pEnd, remap(getCyclePercent(i, t), swingStart, heelStrikeStart));
        } else if (nextPhase == Phase::HeelStrike) {

            P3D pFinal = heel->getEEWorldPos() + getP3D(robot->getHeading() * V3D(robot->getForward() * heelToeDistance));
            pFinal[1] = toeHeight + gui::SimpleGroundModel::getHeight(pFinal);

            toeStrikeTarget[i] = pFinal;

            P3D pHeel = heel->getEEWorldPos();
            pHeel[1] = heelHeight + gui::SimpleGroundModel::getHeight(pHeel);
            heelStartSet[i] = false;
            return pHeel;
        }
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

    void setArmAngles(int armIdx) {
        dVector q;
        gcrr.getQ(q);
        // need to map armIdx 1 to 0 and vice-versa, or maybe not? Use for the swing sync
        double speed_direction = planner->speedForward;
        int syncedLegIdx;
        // Need a smooth transition
        if ((armIdx == 0 && speed_direction >= 0) || (armIdx == 1 && speed_direction < 0)) {
            syncedLegIdx = 1;
        } else {
            syncedLegIdx = 0;
        }
        double cyclePercent = getCyclePercent(syncedLegIdx, t);
        double phase = getPhase(syncedLegIdx, t);

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

    void advanceInTime(double deltaT) override {
        double dt = deltaT;
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
                    drawSphere(toeStrikeTarget[leg], 0.02, *shader, {0, 1, 0});
                    drawSphere(toeTargets[leg], 0.02, *shader, {0, 0, 1});
                    drawSphere(heelTargets[leg], 0.02, *shader, {0, 0, 1});
            }
        }

        for(const auto&[p, r, c] : drawList)
            drawSphere(p, r, *shader, c);

        drawEnvMap(*shader);

        drawList.clear();

//        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco