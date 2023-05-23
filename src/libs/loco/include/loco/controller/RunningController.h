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
    std::shared_ptr<LeggedRobot> robot;
    std::shared_ptr<IK_Solver> ikSolver = nullptr;
    GeneralizedCoordinatesRobotRepresentation gcrr;
    dVector qDefault;

    double heelHeight;
    double toeHeight;

    const double pi = 3.14159265358979323846;
    const double twoPi = pi * 2.0;
        
    double t = 0;
    double cycleLength = 60.0 / 75.0; // in seconds
    // double cycleLength = 1.0;
    double stanceStart = 0.0; // in percent
    double swingStart = 0.35; // in percent
    double heelStrikeStart = 0.95; // in percent

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
    std::array<double, 2> ankleAngleStarts = {0.0, 0.0};
    std::array<V3D, 2> defaultHeelToToe;
    double shoulderAmplitudeDegree = 5.0; // in degree
    double shoulderAmplitude = shoulderAmplitudeDegree * twoPi / 360.0;
    double shoulderMin = shoulderAmplitude;
    double shoulderMax = -shoulderAmplitude; // negative rotation: moves arm forward
    Trajectory1D shoulderJointTrajectory;

    double elbowMin = -0.25;
    double elbowMax = -0.45;
    Trajectory1D elbowJointTrajectory;

    Trajectory1D hipTraj;
    Trajectory1D kneeTraj;
    // 1: shoulder sagittal plane, 2: elbow saggital plane, 3: shoulder torsion
    std::array<std::array<int, 3>, 2> armJointIndices;

    // for upper body
    int spineIdx;
    double spineAmplitudeDegree = 2.0;
    double spineAmplitude = toRad(spineAmplitudeDegree);
    int neckIdx;

    enum Phase {Stance, Swing, HeelStrike};
    int its = 0;
    double baseHeight = 0.88;
    Trajectory1D verticalOffsetTraj;
    Trajectory3D toeTrajectoryHealStrike;
public:
    /**
     * constructor
     */
    explicit RunningController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner), gcrr(planner->robot) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);
        gcrr.getQ(qDefault);
        
        // set height of bob
        /*
        dVector q = qDefault;
        q(1) = baseHeight;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
        */
        auto toe = robot->getLimb(0);
        auto heel = robot->getLimb(2);

        
        heelToeDistance = V3D(heel->getEEWorldPos(), toe->getEEWorldPos()).norm();
        for (int i : {0, 1}) {
            auto toe = robot->getLimb(i);
            auto heel = robot->getLimb(i + 2);
            defaultHeelToToe[i] = V3D(heel->getEEWorldPos(), toe->getEEWorldPos());
        }

        heelHeight = (heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld)[1];
        toeHeight = (toe->limbRoot->getWorldCoordinates() + toe->defaultEEOffsetWorld)[1];
        
        heelHeight = 0.018125;
        toeHeight = 0.008125;
        std::cout << heelHeight << "\n" << toeHeight << std::endl;
        for (int i : {0, 1}) {
            P3D pToes = robot->getLimb(i)->getEEWorldPos();
            P3D pHeel = robot->getLimb(i + 2)->getEEWorldPos();
            //pToes.
            // stanceTargets[i] = pToes;
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
        /*
        for (int i : {0, 1}) {
            setToesToFloor(i);
            setToeTarget(i, toeTargets[i]);
            setHeelToFloor(i);
            setHeelTarget(i, heelTargets[i]);
        }
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        */
        std::array lLegJoints = {"lHip_1", "lKnee", "lAnkle_1", "lToeJoint"};
        std::array rLegJoints = {"rHip_1", "rKnee", "rAnkle_1", "rToeJoint"};
        for (int i = 0; i < lLegJoints.size(); ++i) {
            int lIdx = robot->getJointIndex(lLegJoints[i]);
            int rIdx = robot->getJointIndex(rLegJoints[i]);
            footJointIndices[0][i] = rIdx;
            footJointIndices[1][i] = lIdx;
        }

        spineIdx = robot->getJointIndex("upperback_y");
        neckIdx = robot->getJointIndex("lowerneck_y");

        std::array lArmJoints = {"lShoulder_1", "lElbow_flexion_extension", "lShoulder_torsion"};
        std::array rArmJoints = {"rShoulder_1", "rElbow_flexion_extension", "rShoulder_torsion"};
        for (int i = 0; i < lArmJoints.size(); ++i) {
            int lIdx = robot->getJointIndex(lArmJoints[i]);
            int rIdx = robot->getJointIndex(rArmJoints[i]);
            armJointIndices[0][i] = rIdx;
            armJointIndices[1][i] = lIdx;
        }

        /*
        hipTraj.addKnot(0.0, toRad(-45));
        hipTraj.addKnot(0.4, toRad(15));
        hipTraj.addKnot(0.8, toRad(-55));
        hipTraj.addKnot(1.0, toRad(-45));
        

        kneeTraj.addKnot(0.0, toRad(35));
        kneeTraj.addKnot(0.4, toRad(25));
        kneeTraj.addKnot(0.8, toRad(120));
        kneeTraj.addKnot(1.0, toRad(35));
       
        
        hipTraj.addKnot(0.0, toRad(-35));
        hipTraj.addKnot(0.4, toRad(20));
        hipTraj.addKnot(0.75, toRad(-60));
        hipTraj.addKnot(1.0, toRad(-35));

        kneeTraj.addKnot(0.0, toRad(20));
        kneeTraj.addKnot(0.15, toRad(50));
        kneeTraj.addKnot(0.4, toRad(25));
        kneeTraj.addKnot(0.75, toRad(135));
        kneeTraj.addKnot(1.0, toRad(20));
        
        kneeTraj.addKnot(0.0, toRad(35));
        kneeTraj.addKnot(0.15, toRad(45));
        kneeTraj.addKnot(0.4, toRad(25));
        kneeTraj.addKnot(0.65, toRad(100));
        kneeTraj.addKnot(0.95, toRad(25));
        kneeTraj.addKnot(1.0, toRad(35));

        hipTraj.addKnot(0.0, toRad(-60));
        hipTraj.addKnot(0.4, toRad(-20));
        hipTraj.addKnot(0.8, toRad(-75));
        hipTraj.addKnot(1.0, toRad(-60));
        */
        // kneeTraj.addKnot(0.0, toRad(0.0));
        verticalOffsetTraj.addKnot(0.0, -0.03);
        verticalOffsetTraj.addKnot(0.15, -0.06);
        verticalOffsetTraj.addKnot(0.4, 0.02);
        verticalOffsetTraj.addKnot(0.65, -0.06);
        verticalOffsetTraj.addKnot(0.9, 0.02);
        verticalOffsetTraj.addKnot(1.0, -0.03);
    }

    /**
     * destructor
     */
    ~RunningController() override = default;

    double toRad(double degree) const {
        return degree * twoPi / 360.0;
    }

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
        targetPos[1] = baseHeight + verticalOffset * 0.9;
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);
        robot->setRootState(targetPos, targetOrientation);
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        

        t += dt;

        /*
        for (int i : {0, 1}) {
            setToesToFloor(i);
            setToeTarget(i, toeTargets[i]);
            setHeelToFloor(i);
            setHeelTarget(i, heelTargets[i]);
        }
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        */
        for (int i : {0, 1}) {
            // makeFootParallelToGround(i);
        }
        //return;
        /*
        for (int i : {0, 1}) {
            setLegAngles(i);
        }
        */
        // use the toe as the targeted endeffector in the stance phase
        // use the heel as the targeted endeffector in the swing phase
        // idx i == 0 <-> right leg
        // before the phase
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);

            double toeAngle;
            switch (currentPhase) {
                break;
                case Phase::Stance:
                    if (isEarlyStance(getCyclePercent(i, t))) {
                        setHeelToFloor(i);
                        setHeelTarget(i, heelTargets[i]);
                    } 
                    setToeTarget(i, toeTargets[i]);
                    toeAngle = getToeTargetAngle(i);
                    updateToeLimits(i, toeAngle - 1e-6, toeAngle + 1e-6);
                    break;
                case Phase::Swing:
                    moveToesBackToDefault(i);
                    setHeelTarget(i, heelTargets[i]);
                    // setDefault(i);
                    if (isLateSwing(getCyclePercent(i, t))) {
                        raiseFootDuringSwing(i);
                    }
                    break;
                case Phase::HeelStrike:
                    setToeTarget(i, toeTargets[i]);
                    setHeelTarget(i, heelTargets[i]);
            }
        }
        ikSolver->solve();
        gcrr.syncGeneralizedCoordinatesWithRobotState();
        /*
        for (int i : {0, 1}) {
            Phase currentPhase = getPhase(i, t);
            if (currentPhase == Phase::Stance) {
                makeToesParallelToGround(i);
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
                initializeSwingPhase(i);
            }
            if (nextPhase == Phase::Swing) {
                // reset toe constraints
                auto toeJoint = robot->getJoint(footJointIndices[i][3]);
                updateToeLimits(i, toeJoint->minAngle, toeJoint->maxAngle);
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

    void setLegAngles(int footIdx){
        dVector q;
        gcrr.getQ(q);
        
        double cyclePercent = getCyclePercent(footIdx, t);
        //cyclePercent = 1.0;
        q(6 + footJointIndices[footIdx][0]) = hipTraj.evaluate_catmull_rom(cyclePercent);
        q(6 + footJointIndices[footIdx][1]) = kneeTraj.evaluate_catmull_rom(cyclePercent);

        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
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
            + velocity * cycleLength * (heelStrikeStart - swingStart)  // where the base will be at the end of the swing phase
            + velocityDir * 0.1 // strike roughly under the COM
        );
        // heelEnds[i].z += 0.1; // just for testing while bob is standing
        
        // heelEnds[i][1] = 0.0; // magic number for running...

        V3D p0 = V3D(heelStarts[i]);
        V3D p3 = V3D(heelEnds[i]);
        V3D p1 = lerp(p0, p3, 0.25);
        V3D p2 = lerp(p0, p3, 0.65);
        p1[1] += 0.5;
        p2[1] += 0.2;
        p2 += velocityDir * 0.3;
        // p2[2] += 0.3; // just for testing while bob is standing
        swingTrajectories[i].clear();
        swingTrajectories[i].addKnot(0.0, p0);
        swingTrajectories[i].addKnot(0.35, p1);
        swingTrajectories[i].addKnot(0.65, p2);
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
            P3D pFinal = heel->getEEWorldPos() + defaultHeelToToe[i];
            // pFinal[1] = toeHeight;
            double cyclePercent = getCyclePercent(i, t);
            return lerp(pInitial, pFinal, remap(cyclePercent, heelStrikeStart, 1.0));
        }
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
        q(6 + footJointIndices[footIdx][2]) = angleTarget;
        
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
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
                    swingStart + 0.25 * (heelStrikeStart - swingStart)
                ), 0.0, 1.0)
        );
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

        double elbowTarget = elbowJointTrajectory.evaluate_catmull_rom(cyclePercent);
        q(6 + armJointIndices[armIdx][1]) = elbowTarget;
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void setSpineAngle() {
        dVector q;
        gcrr.getQ(q);

        double cyclePercent = getCyclePercent(0, t);
        double spineTarget = spineAmplitude * sin(twoPi * cyclePercent);
        q(6 + spineIdx) = spineTarget;
        q(6 + neckIdx) = -spineTarget; // so that bob looks straight ahead
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