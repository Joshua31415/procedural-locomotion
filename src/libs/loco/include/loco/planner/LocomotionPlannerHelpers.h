#pragma once

#include <crl-basic/gui/renderer.h>
#include <crl-basic/utils/trajectory.h>
#include <loco/planner/BodyFrame.h>
#include <loco/planner/FootFallPattern.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>
#include <loco/robot/RBUtils.h>

namespace crl::loco {

class LimbMotionProperties {
public:
    double contactSafetyFactor = 0.7;

    // p: this trajectory controlls how fast a foot lifts and how fast it sets
    // back down, encoded as a function of swing phase
    Trajectory1D swingFootHeightTraj;
    Trajectory1D swingArmHeightTraj;
    Trajectory1D swingShoulderHeightTraj;
    // p: given the total step length for a limb, ffStepLengthRatio controls the
    // stance phase when the limb should be right below the hip/shoulder (e.g.
    // default, or zero step length configuration) Should this be per limb?
    double ffStancePhaseForDefaultStepLength = 0.5;

    // to account for a non-zero foot size, we add an offset to the swing foot
    // height. This will allow us to control how aggressively the robot steps
    // down, and it will allow firm contacts to be established
    Trajectory1D swingHeightOffsetTrajDueToFootSize;

    double swingFootHeight = 0.1;
    double swingArmHeight = 0.1;
    double swingShoulderHeight = 0.1;
    double armreserveHeight = 0.085;

    //x and z here are expressed in a heading-independent coordinate frame
    double stepWidthOffsetX = 0.7;
    double stepWidthOffsetZ = 1.0;

    LimbMotionProperties() {
        // p: this trajectory should be parameterized...
        // TODO: change trajectory of the foot here

        for(int i = 0; i < 100; ++i){
            swingFootHeightTraj.addKnot(i * 0.7 / 100, std::sin(3.14159265358979323846 * i / 100));
        }


        // Arm Initial Curve
        swingArmHeightTraj.addKnot(0, 0);
        swingArmHeightTraj.addKnot(0.5, -0.2);
        swingArmHeightTraj.addKnot(1.0, 0);

        // Shoulder - small jitters around the horizon
//        swingShoulderHeightTraj.addKnot(0, 0);
//        swingShoulderHeightTraj.addKnot(0.25, -0.001);
//        swingShoulderHeightTraj.addKnot(0.75, 0.001);
//        swingShoulderHeightTraj.addKnot(1.0, 0);

        // Shoulder - accompanying arm swing
        swingShoulderHeightTraj.addKnot(0, 0);
        swingShoulderHeightTraj.addKnot(0.5, -0.001);
        swingShoulderHeightTraj.addKnot(1.0, 0);

        swingHeightOffsetTrajDueToFootSize.addKnot(0, 1.0);
        swingHeightOffsetTrajDueToFootSize.addKnot(0.5, 1.0);
        // when the transition from swing to stance happens, in order to make
        // sure a firm contact is established, the contactSafetyFactor(default = 0.7)
        //here makes the contact be pretty firm.
        swingHeightOffsetTrajDueToFootSize.addKnot(1.0, 0.7);
    }
};

class PlannedLimbContact {
public:
    //the time the contact phase starts
    double tStart = 0;
    //the time it ends
    double tEnd = 0;
    //and the position where the limb is/will be in contact with the environment
    P3D contactLocation;
    //we will also add a flag here that tells us if this is a current position (i.e. fixed), or a planned one
    bool isFixed = false;
    PlannedLimbContact(double tStart, double tEnd, const P3D& pos, bool isFixed = false) {
        this->tStart = tStart;
        this->tEnd = tEnd;
        this->contactLocation = pos;
        this->isFixed = isFixed;
    }
};

/**
        This class stores a sequence of planned foot falls for each of the robot's limbs.
        Useful bits of information, such as the target stepping location at a particular
        moment in time can be easily retrieved.
    */
class FootstepPlan {
public:
    //this is a pointer to the contact plan manager which corresponds to the sequnce of foot steps that is planned here.
    //Whomever populates the footstep plan should also set the contact plan manager adequately;
    //the two are closely link, as they store complementary information
    ContactPlanManager* cpm = nullptr;
    std::map<const std::shared_ptr<RobotLimb>, DynamicArray<PlannedLimbContact>> footSteps;

    //if the limb is in stance at time t, we'll be returning the current planned
    //position of the contact point; if the limb is in swing at time t, we'll
    //be returning the planned contact position for the end of the swing phase
    P3D getCurrentOrUpcomingPlannedContactLocation(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return footSteps[limb][i].contactLocation;
        }
        return P3D();
    }

    PlannedLimbContact* getCurrentOrUpcomingPlannedLimbContact(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return &footSteps[limb][i];
        }

        return nullptr;
    }

    int getIndexOfCurrentOrUpcomingPlannedLimbContact(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return i;
        }

        return -1;
    }

    //given world coordinates for the step locations, generate continuous trajectories for each of a robot's feet
    Trajectory3D generateLimbTrajectory(const std::shared_ptr<RobotLimb>& limb, const LimbMotionProperties& lmp, double tStart, double tEnd, double dt,
                                        double groundHeight = 0) {
        double t = tStart;
        Trajectory3D traj;

        //always start the trajectory from the current location of the robot
        V3D startingEEPos = V3D(limb->getEEWorldPos());
        traj.addKnot(t, startingEEPos);

        t += dt;

        while (t < tEnd) {
            ContactPhaseInfo cpi = cpm->getCPInformationFor(limb, t);

            if (cpi.isStance()) {
                double tEndOfStance = t + cpi.getTimeLeft();
                // in stance, we want the foot to not slip, while keeping to
                // the ground...
                V3D eePos = traj.getKnotValue(traj.getKnotCount() - 1);
                eePos.y() = groundHeight + limb->ee->defaultHeight;
                if(limb->name == "lLowerArm" || limb->name == "rLowerArm") {
                    eePos.y() += 0.085; //armreserveHeight
                }
                while (t <= tEndOfStance && t < tEnd) {
                    traj.addKnot(t, eePos);
                    t += dt;
                }
            } else {
                double tEndOfSwing = t + cpi.getTimeLeft();

                V3D finalEEPos = V3D(getCurrentOrUpcomingPlannedContactLocation(limb, tEndOfSwing));

                // when we first transition to a swing phase (at some time
                // sample t_i), this could be at the very beginning of it,
                // or it could have been a "little while ago" and so the
                // position of the swing foot at this moment in time needs
                // to account for this sampling-induced variation...
                bool firstTimeStepInSwingPhase = (cpi.getDuration() - cpi.getTimeLeft()) < dt;

                // plan motion for the entire swing phase here to avoid
                // computing redundant information...
                while (t <= tEndOfSwing) {
                    ContactPhaseInfo cpiSwing = cpm->getCPInformationFor(limb, t);

                    double factor = 1.0;
                    if (firstTimeStepInSwingPhase)
                        factor = (cpi.getDuration() - cpi.getTimeLeft()) / dt;
                    firstTimeStepInSwingPhase = false;

                    V3D oldEEPos = traj.getKnotValue(traj.getKnotCount() - 1);
                    // now, we have the remainder of the swing phase to go
                    // from the old step position to the final stepping
                    // location. Based on this we know how much we should be
                    // travelling over a time window dt...
                    double dTimeStep = 1.0;
                    // the -0.05 thing here means we want the swing foot to
                    // reach its target 0.05s before the swing phase ends
                    if (cpiSwing.getTimeLeft() - 0.05 > dt)
                        dTimeStep = dt / (cpiSwing.getTimeLeft() - 0.05) * factor;

                    V3D deltaStep = dTimeStep * (finalEEPos - oldEEPos);
//                    deltaStep[0] = 0.0;
                    //deltaStep[2] = 0.0;
                    V3D eePos = oldEEPos + deltaStep;
                    // for the leg situation
                    if(limb->name == "lLowerLeg" || limb->name == "rLowerLeg") {
                        // add ground height + ee size as offset...
                        // Adapt from ankle_movement branch
                        eePos.y() = groundHeight + lmp.swingFootHeightTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed()) * lmp.swingFootHeight +
                                    limb->ee->defaultHeight;
                    }
                    else if(limb->name == "lLowerArm" || limb->name == "rLowerArm") {
                        // TODO: Try better curve for the task
                        // Method one:
                        // deltaStep = (cpiSwing.getTimeLeft() * 1.2 - 0.002) * (finalEEPos - oldEEPos);
                        // eePos = oldEEPos + deltaStep;
                        // Method two:
                        // eePos = oldEEPos - lmp.swingArmHeightTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed()) * 5 * (finalEEPos - oldEEPos);
                        eePos.y() = groundHeight + lmp.swingArmHeightTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed()) * lmp.swingFootHeight +
                                    limb->ee->defaultHeight + 0.085; // armreserveHeight
                    }
                    else {
                        eePos.y() = groundHeight + lmp.swingShoulderHeightTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed()) * lmp.swingShoulderHeight +
                                    limb->ee->defaultHeight;
                    }
                    traj.addKnot(t, eePos);
                    t += dt;
                }
            }
        }

        return traj;
    }
};

/*
        The body frame motion represents the *average* linear trajectory and heading for a robot's trunk. 
        Roll, pitch and parasitic components of motion (e.g. transient fluctuations in yaw / translation 
        of COM arising from reactions to small perturbations or planned body sway, e.g. side to side motion during a 
        stride) are not captured by the motion of the body frame.
    */
class bFrameReferenceMotionPlan {
private:
    //0-2: xyz coords of position, in world coords
    //3: heading
    typedef Eigen::Matrix<double, 4, 1> bFrameState;

    void generate(const bFrameState& startingbFrameState) {
        double headingAngle = startingbFrameState[3];
        P3D pos(startingbFrameState[0], startingbFrameState[1], startingbFrameState[2]);
        double vForward = targetForwardSpeed;
        double vSideways = targetSidewaysSpeed;
        double turningSpeed = targetTurngingSpeed;

        bFramePosTrajectory.clear();
        bFrameHeadingTrajectory.clear();
        bFrameVels.clear();

        double t = tStart;

        // generate trajectories that capture the motion of the robot's body frame...
        while (t < tEnd) {
            Quaternion heading = getRotationQuaternion(headingAngle, V3D(0, 1, 0));
            pos.y = targetbFrameHeight;

            bFramePosTrajectory.addKnot(t, V3D(pos));
            bFrameHeadingTrajectory.addKnot(t, headingAngle);

            bFrameVels.addKnot(t, V3D(vForward, vSideways, turningSpeed));

            vForward = targetForwardSpeed;
            vSideways = targetSidewaysSpeed;
            turningSpeed = targetTurngingSpeed;

            // TODO: Trajectory Planning
            //
            // update "pos" and "headingAngle".
            // compute nextstep's "pos" and "headingAngle" by integrating "vForward", "vSideways" and "turningSpeed".
            //
            // Hints:
            // - quaternion heading = Roty(headingAngle). (we use y-up world frame coordinate)
            // - you can get the robot's forward direction vector from robot->forward
            // - you can get the robot's sideways direction vector by RBGlobals::worldUp.cross(robot->forward)

            pos = pos + dt * vForward * (heading * robot->getForward()) + dt * vSideways * RBGlobals::worldUp.cross(heading * robot->getForward());
            headingAngle = headingAngle + dt * turningSpeed;

            t += dt;
        }
    }

public:
    //keep track of start, end and sampling rate for the motion trajectory
    double tStart = 0.0;
    double tEnd = 1.0;
    double dt = 1 / 30.0;

    //high level parameters that govern the resulting motion
    double targetForwardSpeed = 0;
    double targetSidewaysSpeed = 0;
    double targetTurngingSpeed = 0;
    double targetbFrameHeight = 0;

    //and the robot we apply this to
    std::shared_ptr<LeggedRobot> robot = nullptr;

    bFrameReferenceMotionPlan(const std::shared_ptr<LeggedRobot>& robot) {
        this->robot = robot;
    }

    // store reference linear motion for the robot's body
    Trajectory3D bFramePosTrajectory;
    //as well as a heading trajectory
    Trajectory1D bFrameHeadingTrajectory;
    //and the profile of turning, forward and sideways velocities that were used
    Trajectory3D bFrameVels;

    bFrameState getBFrameStateFromRBState(const RBState& rbState) {
        bFrameState s;
        BodyFrame bFrame(rbState.pos, rbState.orientation);

        s[0] = bFrame.p[0];
        s[1] = bFrame.p[1];
        s[2] = bFrame.p[2];
        s[3] = bFrame.h;
        return s;
    }

    bFrameState getBFrameStateAt(double t) {
        bFrameState s;
        P3D p = P3D() + bFramePosTrajectory.evaluate_linear(t);
        double h = bFrameHeadingTrajectory.evaluate_linear(t);
        V3D vels = bFrameVels.evaluate_linear(t);
        s[0] = p[0];
        s[1] = p[1];
        s[2] = p[2];
        s[3] = h;
        return s;
    }

    // compute the body frame position and heading based on the
    // position/orientation of the robot's trunk
    bFrameState getInitialConditionsFromCurrentTrunkState() {
        bFrameState initialBFrameState = getBFrameStateFromRBState(robot->getTrunk()->getState());
        return initialBFrameState;
    }

    bool hasGeneratedMotionTrajectory() {
        return bFramePosTrajectory.getKnotCount() > 0;
    }

    //generate the bFrame reference trajectory starting from the current state of the robot's trunk...
    void generateTrajectory() {
        generate(getInitialConditionsFromCurrentTrunkState());
    }

    void populateFootstepPlan(FootstepPlan& fsp, const LimbMotionProperties& lmProps, ContactPlanManager* cpm, double groundHeight = 0) {
        fsp.cpm = cpm;
        double tTiny = 0.0001;
        for (uint i = 0; i < robot->getLimbCount(); i++) {
            const auto& limb = robot->getLimb(i);
            fsp.footSteps[limb].clear();
            ContactPhaseInfo cpi = cpm->getCPInformationFor(limb, tStart);
            double t = tStart;
            if (cpi.isStance()) {
                P3D pos = limb->getEEWorldPos();
                pos.y = groundHeight + limb->ee->radius * lmProps.contactSafetyFactor;  // account for the size of the ee
                fsp.footSteps[limb].push_back(PlannedLimbContact(tStart, tStart + cpi.getTimeLeft(), pos, true));
                t = tStart + cpi.getTimeLeft() + tTiny;
            }

            while (t < tEnd) {
                ContactPhaseInfo cpiSwing = cpm->getCPInformationFor(limb, t);
                //when we get here, at time t, the foot is in swing, so find the start of the next contact phase
                if (cpiSwing.isSwing() == false) {
                    Logger::consolePrint(
                        "ERROR, ERROR, at this point the limb should be in "
                        "swing, but isn't");
                    assert(cpiSwing.isStance());
                }

                t += cpiSwing.getTimeLeft();
                ContactPhaseInfo cpiStance = cpm->getCPInformationFor(limb, t + tTiny);
                if (cpiStance.isStance() == false) {
                    Logger::consolePrint(
                        "ERROR, ERROR, at this point the limb should be in "
                        "stance, but isn't");
                    assert(cpiStance.isSwing());
                }

                double tStart = t;
                double tEnd = tStart + cpiStance.getDuration();
                t = tEnd + tTiny;

                //this is the moment in time where we'd like the limb to be right under its hip
                double tMidStance = tStart + cpiStance.getDuration() * lmProps.ffStancePhaseForDefaultStepLength;
                //so, compute the location of the body frame at that particular moment in time...
                double bFrameHeadingAngle = bFrameHeadingTrajectory.evaluate_linear(tMidStance);
                P3D bFramePos = P3D() + bFramePosTrajectory.evaluate_linear(tMidStance);

                V3D defaultEEOffset = limb->defaultEEOffset;
                defaultEEOffset[0] *= lmProps.stepWidthOffsetX;
                defaultEEOffset[2] *= lmProps.stepWidthOffsetZ;

                P3D pos = bFramePos + getRotationQuaternion(bFrameHeadingAngle, V3D(0, 1, 0)) * defaultEEOffset;

                pos.y = groundHeight + limb->ee->radius * lmProps.contactSafetyFactor;  // account for the size of the ee

                fsp.footSteps[limb].push_back(PlannedLimbContact(tStart, tEnd, pos, false));
            }
        }
    }
};

}  // namespace crl::loco
