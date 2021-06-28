#pragma once

#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/Robot.h>

namespace crl {

struct IK_EndEffectorTargets {
    RB *rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(Robot *robot) : robot(robot) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(RB *rb, P3D p, P3D target) {
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

            // get current generalized coordinates of the robots

            // TODO: Ex.2-2 Inverse Kinematics
            //
            // update generalized coordinates of the robot by solving IK.

            // remember, we don't update base pose since we assume it's already at
            // the target position and orientation
            dVector deltaq(q.size() - 6);
            deltaq.setZero();

            dVector J(q.size() - 6);
            J.setZero();
            // TODO: here, compute deltaq using GD, Newton, or Gauss-Newton.
            // end effector targets are stored in endEffectorTargets vector.
            for (int j = 0; j < endEffectorTargets.size(); j++) {
                Matrix dpdq;

                gcrr.estimate_linear_jacobian(endEffectorTargets[j].p,
                                              endEffectorTargets[j].rb, dpdq);

                double gamma = 1.11;
                Matrix Jacobi = dpdq.block(0, 6, 3, q.size() - 6);
               
           
                //gradient descent
               
                J = gamma *
                    Jacobi.transpose().eval() * 
                    V3D(gcrr.getWorldCoordinates(endEffectorTargets[j].p,
                                                     endEffectorTargets[j].rb),
                            endEffectorTargets[j].target);
              
 /*
                //gauss-newton
                Matrix J_T = Jacobi.transpose();
                Matrix J_IT = J_T * Jacobi;
                Matrix i = Matrix::Identity(q.size() - 6, q.size() - 6);
               
                Matrix J_I = J_IT.ldlt().solve(i);

                P3D t = endEffectorTargets[j].target;
                P3D FK = gcrr.getWorldCoordinates(endEffectorTargets[j].p,
                                                  endEffectorTargets[j].rb);
                J = J_I * J_T * V3D(t - FK);
     */
               int q1 = gcrr.getQIdxForJoint(endEffectorTargets[j].rb->pJoint);
               int q2 = gcrr.getParentQIdxOf(q1);
               int q3 = gcrr.getParentQIdxOf(q2);
               
               deltaq[q1 - 6] += J[q1 - 6];         
               deltaq[q2 - 6] += J[q2 - 6];
               deltaq[q3 - 6] += J[q3 - 6];
               
            }
 
            // Hint:
            // - use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
            // - if you already implemented analytic Jacobian, you can use gcrr.compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq)
            // - don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
            // - when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
            //   see https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

            // TODO: your implementation should be here.

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
    Robot *robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
};

}  // namespace crl