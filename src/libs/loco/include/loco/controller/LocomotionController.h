#pragma once

#include <crl-basic/gui/renderer.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/kinematics/IK_Solver.h>

namespace crl::loco {

/**
 * Abstract class for legged locomotion controllers. It has trajectory planner
 * as a member. Derived class of this class should implement planning and
 * control logic.
 */
class LocomotionController {
public:
    const double pi = 3.14159265358979323846;
    const double twoPi = pi * 2.0;

    std::shared_ptr<LocomotionTrajectoryPlanner> planner;
    GeneralizedCoordinatesRobotRepresentation gcrr;
    std::shared_ptr<LeggedRobot> robot;
    std::shared_ptr<IK_Solver> ikSolver;

    //Vector of point/radius/color
    std::vector<std::tuple<P3D, double, V3D>> drawList{};

    std::array<Trajectory3D, 2> swingTrajectories{};

    std::array<P3D, 2> toeTargets{};
    std::array<P3D, 2> heelTargets{};
    std::array<P3D, 2> heelStarts{};
    std::array<P3D, 2> heelEnds{};

    Trajectory1D shoulderJointTrajectory{};

        double t;

        double &cycleLength;
        double &stanceStart;
        double &swingStart;
        double &heelStrikeStart;

        const double earlyStanceDuration = 0.5;

        enum Phase { Stance, Swing, HeelStrike };

        // 1: shoulder sagittal plane, 2: elbow saggital plane, 3: shoulder torsion
        std::array<std::array<int, 3>, 2> armJointIndices{};

        const double spineAmplitude;
        const double pelvisAmplitude_x;
        const double pelvisAmplitude_z;
        int spineIdx;
        int neckIdx;
        std::array<int, 3> pelvisIdx{};

        // hip, knee, ankle1, toe, ankle2
        std::array<std::array<int, 5>, 2> footJointIndices{};
        double heelHeight;
        double toeHeight;
        double heelToeDistance;
public:
    LocomotionController(const std::shared_ptr<LocomotionTrajectoryPlanner>& planner, double &cycleLength, double &stanceStart, double &swingStart,
                         double &heelStrikeStart, double shoulderMax, double shoulderMin, double spineAmplitudeDegree, double pelvisAmplitudeDegree_x,
                         double pelvisAmplitudeDegree_z)
        : planner(planner),
          gcrr(planner->robot),
          cycleLength(cycleLength),
          stanceStart(stanceStart),
          swingStart(swingStart),
          heelStrikeStart(heelStrikeStart),
          spineAmplitude(toRad(spineAmplitudeDegree)),
          pelvisAmplitude_x(toRad(pelvisAmplitudeDegree_x)),
          pelvisAmplitude_z(toRad(pelvisAmplitudeDegree_z))
    {
        
        t = 0.0;

        robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);

        auto toe = robot->getLimb(0);
        auto heel = robot->getLimb(2);

        heelToeDistance = V3D(heel->getEEWorldPos(), toe->getEEWorldPos()).norm();
        heelHeight = (heel->limbRoot->getWorldCoordinates() + heel->defaultEEOffsetWorld)[1];
        toeHeight = (toe->limbRoot->getWorldCoordinates() + toe->defaultEEOffsetWorld)[1];

        std::array lLegJoints = {"lHip_1", "lKnee", "lAnkle_1", "lToeJoint", "lAnkle_2"};
        std::array rLegJoints = {"rHip_1", "rKnee", "rAnkle_1", "rToeJoint", "rAnkle_2"};
        for (int i = 0; i < lLegJoints.size(); ++i) {
            int lIdx = robot->getJointIndex(lLegJoints[i]);
            int rIdx = robot->getJointIndex(rLegJoints[i]);
            footJointIndices[0][i] = rIdx;
            footJointIndices[1][i] = lIdx;
        }

        std::array lArmJoints = {"lShoulder_1", "lElbow_flexion_extension", "lShoulder_torsion"};
        std::array rArmJoints = {"rShoulder_1", "rElbow_flexion_extension", "rShoulder_torsion"};
        for (int i = 0; i < lArmJoints.size(); ++i) {
            int lIdx = robot->getJointIndex(lArmJoints[i]);
            int rIdx = robot->getJointIndex(rArmJoints[i]);
            armJointIndices[0][i] = rIdx;
            armJointIndices[1][i] = lIdx;
        }

        spineIdx = robot->getJointIndex("upperback_y");
        neckIdx = robot->getJointIndex("lowerneck_y");

        pelvisIdx[0] = robot->getJointIndex("lowerback_x");
        pelvisIdx[1] = robot->getJointIndex("lowerback_y");
        pelvisIdx[2] = robot->getJointIndex("lowerback_z");
    }

    virtual ~LocomotionController()= default;;

    /**
     * Generate motion trajectory with timestep size dt.
     */
    void generateMotionTrajectories(double dt = 1.0 / 30) {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    /**
     * Compute and apply control signal with timestep size dt.
     */
    virtual void computeAndApplyControlSignals(double dt) = 0;

    /**
     * Call this function after applying control signal.
     */
    void advanceInTime(double deltaT) {
        planner->advanceInTime(deltaT);
    }

    /**
     * Draw control options to ImGui.
     */
    virtual void drawControllerOptions() {
        ImGui::Text("No option available");
    };

    /**
     * Draw some useful information for debugging.
     * e.g. contact, velocity, acceleration etc.
     */
    virtual void drawDebugInfo(gui::Shader* shader) = 0;

    /**
     * Plot some useful information for debugging.
     * e.g. joint torque etc.
     */
    virtual void plotDebugInfo() = 0;

    /**
     * @brief Computes catmull rom interpolation of swing trajectory of heel
     * @param i leg index
     * @param dt delta time of simulation
     * @return interpolated heel position at time t+dt
     */
    [[nodiscard]] P3D computeHeelTargetSwing(int i, double dt) {
        double cyclePercent = getCyclePercent(i, t+dt);

        return getP3D(swingTrajectories[i].evaluate_catmull_rom(remap(cyclePercent, swingStart, heelStrikeStart)));
    }


    [[nodiscard]] double toRad(double degree) const {
        return degree * twoPi / 360.0;
    }
    
    // remap t in [a, b] to [0, 1]
    static double remap(double time, double a, double b) {
        return (time - a) / (b - a);
    }

    template <typename T>
    T lerp(T a, T b, double time) {
        return a * (1.0 - time) + b * time;
    }

    [[nodiscard]] double getCyclePercent(int i, double time) const {
        return std::fmod(i * 0.5 * cycleLength + time, cycleLength) / cycleLength;
    }

    [[nodiscard]] bool isStance(double cyclePercent) const {
        return stanceStart <= cyclePercent && cyclePercent < swingStart;
    }

    [[nodiscard]] bool isEarlyStance(double cyclePercent) const {
        return isStance(cyclePercent) && (cyclePercent < (stanceStart + swingStart) * earlyStanceDuration);
    }

    [[nodiscard]] bool isSwing(double cyclePercent) const {
        return swingStart <= cyclePercent && cyclePercent < heelStrikeStart;
    }

    [[nodiscard]] bool isHeelStrike(double cyclePercent) const {
        return heelStrikeStart <= cyclePercent && cyclePercent < 1.0;
    }

    [[nodiscard]] Phase getPhase(int i, double time) const {
        double cyclePercent = getCyclePercent(i, time);
        if (isStance(cyclePercent)) {
            return Phase::Stance;
        } else if (isSwing(cyclePercent)) {
            return Phase::Swing;
        } else if (isHeelStrike(cyclePercent)) {
            return Phase::HeelStrike;
        }
    }

    void setHeelTarget(int i, P3D pTarget, double weight=1) {
        auto heel = robot->getLimb(i + 2);
        ikSolver->addEndEffectorTarget(heel->eeRB, heel->ee->endEffectorOffset, pTarget, weight);
    }

    void setToeTarget(int i, P3D pTarget, double weight=1) {
        auto foot = robot->getLimb(i);
        ikSolver->addEndEffectorTarget(foot->eeRB, foot->ee->endEffectorOffset, pTarget, weight);
    }

    void setSpineAngle() {
        dVector q;
        gcrr.getQ(q);

        double cyclePercent = getCyclePercent(0, t);
        double spineTarget = spineAmplitude * sin(twoPi * cyclePercent - 3/5 * pi);
        q(6 + spineIdx) = spineTarget;
        q(6 + neckIdx) = -spineTarget;  // so that bob looks straight ahead
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }


    void setAnkleAngleDuringHeelStrike(int footIdx) {
        dVector q;
        gcrr.getQ(q);
        double angle = 0;
        for (auto i : {0, 1}) {
            angle += q(footJointIndices[footIdx][i] + 6);
        }
        double cyclePercent = getCyclePercent(footIdx, t);
        double angleTarget = -angle;
        double angleCurrent = q(6 + footJointIndices[footIdx][2]);

        q(6 + footJointIndices[footIdx][2]) = lerp(angleCurrent, angleTarget, remap(cyclePercent, heelStrikeStart, 1.0));
        gcrr.setQ(q);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void drawGround(gui::Shader *shader) const {
        if(!gui::SimpleGroundModel::isFlat)
            gui::SimpleGroundModel::groundUneven.draw(*shader, V3D(0, 0, 0));
    }

    void drawEnvironment(gui::Shader *shader) const {
        drawEnvMap(*shader, planner->getTargetTrunkPositionAtTime(t));
    }
};

}  // namespace crl::loco
