#pragma once

#include <loco/robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <loco/robot/Robot.h>

namespace crl::loco {

struct IK_EndEffectorTargets {
    std::shared_ptr<RB> rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(const std::shared_ptr<Robot> &robot) : robot(robot) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(const std::shared_ptr<RB> &rb, P3D p, P3D target) {
        endEffectorTargets.push_back(IK_EndEffectorTargets());
        endEffectorTargets.back().rb = rb;
        endEffectorTargets.back().p = p;
        endEffectorTargets.back().target = target;
    }

    void solve(int nSteps = 10) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);

        for (uint i = 0; i < nSteps; i++) {
            dVector q;
            gcrr.getQ(q);


            dVector deltaq(q.size() - 6);
            deltaq.setZero();

            for (auto &t: endEffectorTargets) {
                // compute error
                P3D pWorld = gcrr.getWorldCoordinates(t.p, t.rb);
                P3D pTarget = t.target;
                V3D error = (V3D(pTarget) - V3D(pWorld));

                // compute Jacobian
                Matrix J;
                gcrr.estimate_linear_jacobian(t.p, t.rb, J);
                J = J.block(0, 6, 3, q.size() - 6).eval();
                // update dq
                deltaq += (J.transpose() * J).ldlt().solve(J.transpose() * error);
            }
            q.tail(q.size() - 6) += deltaq;

            // now update gcrr with q
            gcrr.setQ(q);
        }

        gcrr.syncRobotStateWithGeneralizedCoordinates();

        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    std::shared_ptr<Robot> robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
};

}  // namespace crl::loco