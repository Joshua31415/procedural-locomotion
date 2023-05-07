#pragma once

#include <loco/robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <loco/robot/Robot.h>

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


namespace crl::loco {

struct IK_EndEffectorTargets {
    std::shared_ptr<RB> rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(const std::shared_ptr<Robot> &robot) : robot(robot) {
        nJoints = robot->getJointCount();
        jointLimits = Matrix(nJoints, 2);
        for (int i = 0; i < nJoints; ++i) {
            auto joint = robot->getJoint(i);
            jointLimits(i, 0) = joint->minAngle;
            jointLimits(i, 1) = joint->maxAngle;
        }
    }

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

    void enforceJointLimits(dVector& q) {
        for (int i = 0; i < nJoints; ++i) {
            double val = q[6 + i];
            double minVal = jointLimits(i, 0);
            double maxVal = jointLimits(i, 1);
            q[6 + i] = std::clamp(val, minVal, maxVal);
        }    
    }

    
    struct CostFunctor {
        bool operator()(const double* const * x, double* residual) const {
            dVector qModified = q;
            for (uint i = 0; i < nJoints; ++i) {
                qModified(6 + i) = x[0][i];
            }
            gcrr.setQ(qModified);

            P3D pWorld = gcrr.getWorldCoordinates(t.p, t.rb);
            V3D error = (V3D(t.target) - V3D(pWorld));

            residual[0] = error(0);
            residual[1] = error(1);
            residual[2] = error(2);
            return true;
        }

        CostFunctor(GeneralizedCoordinatesRobotRepresentation& gcrr, const IK_EndEffectorTargets& t, uint nJoints) : gcrr(gcrr), t(t), nJoints(nJoints) {
            gcrr.getQ(q);
        }

        GeneralizedCoordinatesRobotRepresentation& gcrr;
        dVector q;
        const IK_EndEffectorTargets& t;
        uint nJoints;
    };


    void solve(int nSteps = 10) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);


        for (auto& t : endEffectorTargets) {
            ceres::DynamicNumericDiffCostFunction<CostFunctor>* cost_function =
                (new ceres::DynamicNumericDiffCostFunction<CostFunctor, ceres::CENTRAL>(new CostFunctor(gcrr, t, nJoints)));
            cost_function->SetNumResiduals(3);
            cost_function->AddParameterBlock(nJoints);


            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            
            dVector qInitial;
            gcrr.getQ(qInitial);

            dVector qNew(qInitial.size() - 6);
            for (uint i = 0; i < qNew.size(); ++i) {
                qNew(i) = qInitial(i + 6);
            }
            
            double* pData = &qNew(0);


            Problem problem;

            problem.AddParameterBlock(pData, nJoints);
            for (int i = 0; i < nJoints; ++i) {
                problem.SetParameterLowerBound(pData, i, jointLimits(i, 0));
                problem.SetParameterUpperBound(pData, i, jointLimits(i, 1));    
            }
            problem.AddResidualBlock(cost_function, nullptr, pData);

            Solver::Summary summary;
            Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";
            
            dVector qFinal = qInitial;
            for (int i = 0; i < nJoints; ++i) {
                qFinal[6 + i] = qNew(i);
            }
            gcrr.setQ(qFinal);
            //double sdasda;
            //std::cin >> sdasda;
        }
        gcrr.syncRobotStateWithGeneralizedCoordinates();
        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    std::shared_ptr<Robot> robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
    Matrix jointLimits;
    int nJoints;
};

}  // namespace crl::loco