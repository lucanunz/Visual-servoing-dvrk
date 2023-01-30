#include "dVKinematics.hpp"

Eigen::Matrix<float, 3, 4> ECM_Jacobian(const Eigen::Vector4f& q){

	Eigen::Matrix<float, 3, 4> J;
	J.setZero();
	
    double q1 = q(0);
	double q2 = q(1);
	double q3 = q(2);
	double q4 = q(3);

	double t2 = cos(q2);
	double t3 = sin(q1);
	double t4 = cos(q1);
	double t5 = sin(q2);
	double t6 = t2*t4;

	//jacobian(0,1) = q3*t2;
	//jacobian(0,2) = t5;
	J(0, 1) = t4;
	J(0, 3) = -t2*t3;
	J(1, 1) = t3;
	J(1, 3) = t6;
	J(2, 0) = 1.0;
	J(2, 3) = t5;

	return J;
}

Eigen::Matrix4f forwardKinematicsECM(const Eigen::Vector4f &q)
{
	Eigen::Matrix4f Ret;
	Eigen::Matrix3f R;
	Eigen::Vector3f p;
	double q1 = q[0];
	double q2 = q[1];
	double q3 = q[2];
	double q4 = q[3];
	double t2 = cos(q1);
	double t3 = sin(q4);
	double t4 = cos(q4);
	double t5 = sin(q1);
	double t6 = sin(q2);
	double t7 = cos(q2);
	R(0, 0) = t2*t4 - t3*t5*t6;
	R(0, 1) = -t2*t3 - t4*t5*t6;
	R(0, 2) = -t5*t7;
	p(0) = -q3*t5*t7;
	R(1, 0) = t4*t5 + t2*t3*t6;
	R(1, 1) = -t3*t5 + t2*t4*t6;
	R(1, 2) = t2*t7;
	p(1) = q3*t2*t7;
	R(2, 0) = -t3*t7;
	R(2, 1) = -t4*t7;
	R(2, 2) = t6;
	p(2) = q3*t6;

	Ret(0, 0) = R(0, 0);
	Ret(0, 1) = R(0, 1);
	Ret(0, 2) = R(0, 2);
	Ret(1, 0) = R(1, 0);
	Ret(1, 1) = R(1, 1);
	Ret(1, 2) = R(1, 2);
	Ret(2, 0) = R(2, 0);
	Ret(2, 1) = R(2, 1);
	Ret(2, 2) = R(2, 2);

	Ret(0, 3) = p(0);
	Ret(1, 3) = p(1);
	Ret(2, 3) = p(2);

	Ret(3, 0) = 0;
	Ret(3, 1) = 0;
	Ret(3, 2) = 0;
	Ret(3, 3) = 1;

	return Ret;
}

Matrix6f PSM_Jacobian(const Vector6f& q){

	Eigen::Matrix<float, 6, 6> J;
	J.setZero();

    const int symSize = 64;
    double t[symSize];
    double q1, q2, q3, q4, q5, q6;
    double l = 0;

    q1 = q(0);
    q2 = q(1);
    q3 = q(2);
    q4 = q(3);
    q5 = q(4);
    q6 = q(5);

    double A0[6][6];

    float t2 = cos(q1);
    float t3 = cos(q4);
    float t4 = sin(q1);
    float t5 = sin(q2);
    float t6 = sin(q4);
    float t7 = sin(q5);
    float t8 = t4*t6;
    float t15 = t2*t3*t5;
    float t9 = t8-t15;
    float t10 = cos(q2);
    float t11 = cos(q5);
    float t12 = q3-1.56E-2;
    float t13 = cos(q6);
    float t14 = sin(q6);
    float t16 = t7*t9*9.1E-3;
    float t17 = t2*t10*t11*9.1E-3;
    float t18 = t5*t11*9.1E-3;
    float t19 = t3*t7*t10*9.1E-3;
    float t20 = t7*t9;
    float t21 = t2*t10*t11;
    float t22 = t20+t21;
    float t23 = t3*t4;
    float t24 = t2*t5*t6;
    float t25 = t23+t24;
    float t35 = t13*t22;
    float t36 = t14*t25;
    float t26 = t35-t36;
    float t27 = t5*t11;
    float t28 = t3*t7*t10;
    float t29 = t27+t28;
    float t30 = t13*t29;
    float t31 = t6*t10*t14;
    float t32 = t30+t31;
    float t33 = t18+t19;
    float t34 = t16+t17;
    float t37 = t2*t10*t12;
    float t38 = t2*t6;
    float t39 = t3*t4*t5;
    float t40 = t38+t39;
    float t41 = t4*t10*t11*9.1E-3;
    float t42 = t7*t40;
    float t49 = t4*t10*t11;
    float t43 = t42-t49;
    float t44 = t2*t3;
    float t47 = t4*t5*t6;
    float t45 = t44-t47;
    float t50 = t13*t43;
    float t51 = t14*t45;
    float t46 = t50-t51;
    float t54 = t7*t40*9.1E-3;
    float t48 = t41-t54;
    float t52 = t9*t11;
    float t65 = t2*t7*t10;
    float t53 = t52-t65;
    float t55 = t4*t10*t12;
    float t56 = t5*t12;
    float t57 = t18+t19+t56;
    float t58 = t5*t7;
    float t64 = t3*t10*t11;
    float t59 = t58-t64;
    float t60 = t11*t40;
    float t61 = t4*t7*t10;
    float t62 = t60+t61;
    float t63 = t4*t10;

    A0[0][0] = t16+t17+t37+l*t26;
    A0[0][1] = -t4*t57-l*t4*t32;
    A0[0][2] = t63;
    A0[0][3] = t5*t34+l*t5*t26-t2*t10*t33-l*t2*t10*t32;
    A0[0][4] = -t25*t33-l*t25*t32-t6*t10*t34-l*t6*t10*t26;
    A0[0][5] = -l*t26*t59-l*t32*t53;
	A0[1][0] = 0.0;
    A0[1][1] = -t4*(t41+t55-t7*t40*9.1E-3)-t2*(t16+t17+t37)-l*t2*t26+l*t4*t46;
    A0[1][2] = -t5;
    A0[1][3] = t4*t10*t34-t2*t10*t48+l*t4*t10*t26+l*t2*t10*t46;
    A0[1][4] = -t25*t48-t34*t45+l*t25*t46-l*t26*t45;
    A0[1][5] = -l*t26*t62+l*t46*t53;
    A0[2][0] = t41-t54+t55-l*t46;
    A0[2][1] = t2*t57+l*t2*t32;
    A0[2][2] = -t2*t10;
    A0[2][3] = t5*(t41-t54)-l*t5*t46-t4*t10*t33-l*t4*t10*t32;
    A0[2][4] = t33*t45+l*t32*t45-t6*t10*t48+l*t6*t10*t46;
    A0[2][5] = l*t32*t62+l*t46*t59;
	A0[3][0] = 0.0;
	A0[3][1] = -t2;
	A0[3][2] = 0.0;
	A0[3][3] = t63;
    A0[3][4] = -t44+t47;
    A0[3][5] = -t60-t61;
    A0[4][0] = -1.0;
	A0[4][1] = 0.0;
	A0[4][2] = 0.0;
	A0[4][3] = -t5;
    A0[4][4] = t6*t10;
    A0[4][5] = t59;
	A0[5][0] = 0.0;
	A0[5][1] = -t4;
	A0[5][2] = 0.0;
	A0[5][3] = -t2*t10;
    A0[5][4] = -t23-t24;
    A0[5][5] = -t52+t65;

    J << A0[0][0], A0[0][1], A0[0][2], A0[0][3], A0[0][4], A0[0][5],
         A0[1][0], A0[1][1], A0[1][2], A0[1][3], A0[1][4], A0[1][5],
         A0[2][0], A0[2][1], A0[2][2], A0[2][3], A0[2][4], A0[2][5],
         A0[3][0], A0[3][1], A0[3][2], A0[3][3], A0[3][4], A0[3][5],
         A0[4][0], A0[4][1], A0[4][2], A0[4][3], A0[4][4], A0[4][5],
         A0[5][0], A0[5][1], A0[5][2], A0[5][3], A0[5][4], A0[5][5];

    return J;
}

Eigen::Matrix4f PSM1DirKin(const Vector6f& q) {

	Eigen::Matrix4f Tbee;
	double q1, q2, q3, q4, q5, q6;
	double l = 0;
	const int symSize = 36;
	double t[symSize];

	q1 = q(0);
	q2 = q(1);
	q3 = q(2);
	q4 = q(3);
	q5 = q(4);
	q6 = q(5);

	t[2] = sin(q1);
	t[3] = cos(q1);
	t[4] = cos(q4);
	t[5] = sin(q2);
	t[6] = sin(q4);
	t[7] = cos(q5);
	t[8] = t[3] * t[6];
	t[9] = t[2] * t[4] * t[5];
	t[10] = t[8] + t[9];
	t[11] = cos(q2);
	t[12] = sin(q5);
	t[13] = cos(q6);
	t[14] = t[10] * t[12];
	t[19] = t[2] * t[7] * t[11];
	t[15] = t[14] - t[19];
	t[16] = sin(q6);
	t[17] = t[3] * t[4];
	t[20] = t[2] * t[5] * t[6];
	t[18] = t[17] - t[20];
	t[21] = t[16] * t[18];
	t[22] = t[5] * t[7];
	t[23] = t[4] * t[11] * t[12];
	t[24] = t[22] + t[23];
	t[25] = q3 - 1.56E-2;
	t[26] = t[2] * t[6];
	t[28] = t[3] * t[4] * t[5];
	t[27] = t[26] - t[28];
	t[29] = t[12] * t[27];
	t[30] = t[3] * t[7] * t[11];
	t[31] = t[29] + t[30];
	t[32] = t[2] * t[4];
	t[33] = t[3] * t[5] * t[6];
	t[34] = t[32] + t[33];
	t[35] = t[16] * t[34];

	Tbee << -t[13] * t[18] - t[15] * t[16], t[7] * t[10] + t[2] * t[11] * t[12], t[21] - t[13] * t[15], t[10] * t[12] * (-9.1E-3) + l*(t[21] - t[13] * t[15]) + t[2] * t[7] * t[11] * 9.1E-3 + t[2] * t[11] * t[25],
		-t[16] * t[24] + t[6] * t[11] * t[13], -t[5] * t[12] + t[4] * t[7] * t[11], -t[13] * t[24] - t[6] * t[11] * t[16], -l*(t[13] * t[24] + t[6] * t[11] * t[16]) - t[5] * t[7] * 9.1E-3 - t[5] * t[25] - t[4] * t[11] * t[12] * 9.1E-3,
		-t[13] * t[34] - t[16] * t[31], t[7] * t[27] - t[3] * t[11] * t[12], t[35] - t[13] * t[31], t[12] * t[27] * (-9.1E-3) + l*(t[35] - t[13] * t[31]) - t[3] * t[7] * t[11] * 9.1E-3 - t[3] * t[11] * t[25],
		0.0, 0.0, 0.0, 1.0;

	return Tbee;

}
