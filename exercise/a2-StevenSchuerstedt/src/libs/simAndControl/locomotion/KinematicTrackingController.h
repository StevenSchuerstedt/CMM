#pragma once

#include <locomotion/LocomotionController.h>
#include <locomotion/LocomotionTrajectoryPlanner.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>
#include <utils/mathUtils.h>
namespace crl {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    LeggedRobot *robot;
    IK_Solver *ikSolver = nullptr;
    

public:
    /**
     * constructor
     */
    KinematicTrackingController(LocomotionTrajectoryPlanner *planner)
        : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = new IK_Solver(robot);
    }

    /**
     * destructor
     */
    virtual ~KinematicTrackingController(void) { delete ikSolver; }

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }


    double getHeight(double x, double y,  double z) {
        double Height = 0.0;
        //is point inside sphere?
        double sphereX = 4.0;
        double sphereZ = 0.0;
        double sphereY = -4.5;
        double sphereR = 5.0;
        //sphere location: P3D(3.0, -2.0, 0.0), radius = 3.0
        double distance = sqrt(pow(x - sphereX, 2) + pow(z - sphereZ,2));
        if (distance <= sphereR) {
            //target inside sphere
            Height = sqrt(pow(sphereR, 2) - pow(x - sphereX, 2) -
                          pow(z - sphereZ, 2)) +
                     sphereY;
        }
     
        double sphereX2 = 0.0;
        double sphereZ2 = 5.0;
        double sphereY2 = -3.5;
        double sphereR2 = 4.0;

         double distance2 = sqrt(pow(x - sphereX2, 2) + pow(z - sphereZ2, 2));
        if (distance2 <= sphereR2) {
            //target inside sphere
            Height = sqrt(pow(sphereR2, 2) - pow(x - sphereX2, 2) -
                          pow(z - sphereZ2, 2)) +
                     sphereY2;
        }

        if (Height < 0.0) return 0.0;
        return Height;
    }

    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos =
            planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(
            planner->getSimTime() + dt);

        targetPos[1] += getHeight(targetPos[0], targetPos[1], targetPos[2]);

        robot->setRootState(targetPos, targetOrientation);

        // now we solve inverse kinematics for each limbs
        for (uint i = 0; i < robot->limbs.size(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(
                robot->limbs[i], planner->getSimTime() + dt);
            
               target[1] += getHeight(target[0], target[1], target[2]);

            ikSolver->addEndEffectorTarget(
                robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
                target);
        }

        ikSolver->solve();
    }

    void advanceInTime(double dt) override { planner->advanceInTime(dt); }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl