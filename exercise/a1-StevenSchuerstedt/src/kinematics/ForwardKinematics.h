#include "Linkage.h"

using Eigen::Vector2d;
using Eigen::Matrix2d;
typedef std::array<Matrix2d, 2> Tensor2x2x2;

/* Returns the 3 points {p0, p1, p2} of the linkage
 * given two angles {a0, a1}
 * Note: p0 is given by the Linkage
 *
 *                  o < p2 = end-effector position
 *                 /
 *                / bar2
 *   a1    bar1  /
 *   o----------o < a2
 *   ^          ^
 *   p0         p1
 *
 */
std::array<Vector2d, 3> forwardKinematics(const Linkage &linkage, const Vector2d &angles) {
    // 1 - Forward Kinematics
    // put your code in this function
    Vector2d p0 = linkage.p0;
    Vector2d p1 = p0 + Vector2d(0, linkage.length[0]);
    Vector2d p2 = p1 + Vector2d(0, linkage.length[1]);
	
	p1.x() = p0.x() + linkage.length[0] * std::cos(angles.x());
	p1.y() = p0.y() + linkage.length[0] * std::sin(angles.x());


	p2.x() = p1.x() + linkage.length[1] * std::cos(angles.x() + angles.y());
	p2.y() = p1.y() + linkage.length[1] * std::sin(angles.x() + angles.y());
    return {p0, p1, p2};
}

Vector2d endEffectorPosition(const Linkage &linkage, const Vector2d &angles) {
    return forwardKinematics(linkage, angles)[2];
}

/* Return the (analytically computed) Jacobian of end-effector position
 * given the angles.
 *
 * Note: end-effector position = forwardKinematics(angles)[2]
 *
 *  d forwardKinematics[2]  <-- end effector position
 *  ----------------------
 *      d angles            <-- wrt. to both angles
 *
 * Hint: this might be useful for IK.
 */
Matrix2d dendEffector_dangles(const Linkage &linkage, const Vector2d &angles) {

    // 2 - Derivatives of Forward Kinematics
    // put your code in this function

    Matrix2d dp2_dangles = Matrix2d::Zero();
	dp2_dangles(0,0) = -linkage.length[1] * std::sin(angles.x() + angles.y()) - linkage.length[0] * std::sin(angles.x());
	dp2_dangles(0,1) = -linkage.length[1] * std::sin(angles.x() + angles.y());
	dp2_dangles(1,0) = linkage.length[1] * std::cos(angles.x() + angles.y()) + linkage.length[0] * std::cos(angles.x());
	dp2_dangles(1,1) = linkage.length[1] * std::cos(angles.x() + angles.y());
    return dp2_dangles;
}

/* Return the (analytically computed) 2nd order derivative of end-effector position
 * given the angles.
 *
 * Note: end-effector position = forwardKinematics(angles)[2]
 *
 *  d^2 forwardKinematics[2]  <-- end effector position
 *  ----------------------
 *      d angles^2            <-- wrt. to both angles
 *
 * Hint: this might be useful for IK.
 */
Tensor2x2x2 ddendEffector_ddangles(const Linkage &linkage, const Vector2d &angles) {

    // 2 - Derivatives of Forward Kinematics
    // put your code in this function
    Tensor2x2x2 tensor;
	long double dJ_00_dTheta1 = -linkage.length[1] * std::cos(angles.x() + angles.y()) - linkage.length[0] * std::cos(angles.x());
	long double dJ_01_dTheta1 = -linkage.length[1] * std::cos(angles.x() + angles.y());
	long double dJ_10_dTheta1 = -linkage.length[1] * std::sin(angles.x() + angles.y()) - linkage.length[0] * std::sin(angles.x());
	long double dJ_11_dTheta1 = -linkage.length[1] * std::sin(angles.x() + angles.y());

	long double dJ_00_dTheta2 = -linkage.length[1] * std::cos(angles.x() + angles.y());
	long double dJ_01_dTheta2 = -linkage.length[1] * std::cos(angles.x() + angles.y());
	long double dJ_10_dTheta2 = -linkage.length[1] * std::sin(angles.x() + angles.y());
	long double dJ_11_dTheta2 = -linkage.length[1] * std::sin(angles.x() + angles.y());

	Matrix2d dJ_dTheta1;
	dJ_dTheta1(0, 0) = dJ_00_dTheta1;
	dJ_dTheta1(0, 1) = dJ_01_dTheta1;
	dJ_dTheta1(1, 0) = dJ_10_dTheta1;
	dJ_dTheta1(1, 1) = dJ_11_dTheta1;

	Matrix2d dJ_dTheta2;
	dJ_dTheta2(0, 0) = dJ_00_dTheta2;
	dJ_dTheta2(0, 1) = dJ_01_dTheta2;
	dJ_dTheta2(1, 0) = dJ_10_dTheta2;
	dJ_dTheta2(1, 1) = dJ_11_dTheta2;

	tensor[0] = dJ_dTheta1;
	tensor[1] = dJ_dTheta2;

    return tensor;
}
