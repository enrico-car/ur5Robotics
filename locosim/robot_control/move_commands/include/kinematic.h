#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__
#include "algebra.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <iostream>

/// @brief params for the ros trajectory topic
typedef struct Trajectory
{
    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<double> times;

} Trajectory;

std::ostream &operator<<(std::ostream &o, const Trajectory &T)
{
    for ( int i=0; i<T.positions.size(); ++i )
    {
        for ( int j=0; j<T.positions[i].size(); ++j )
            o << T.positions[i][j] << " ";
        o << T.times[i] << "\n";
    }
    o << std::endl;

    return o;
}

typedef enum CurveType
{
    BEZIER,
    LINE

} CurveType;

/// @brief Class to handle the kinematic interpolation
class Kinematic
{
private:
    static constexpr double posErrorBoundary = 0.1;
    static constexpr double intermediatePrecision = 0.005;
    static constexpr double finalPrecision = 0.001;
    static constexpr double max_dq = 0.032;

    static Vector6 getDirectionStep(const Matrix4 &actualPose, const Matrix4 &desiredPose)
    {
        std::complex<double> complex(0, 0);
        Matrix4 T = desiredPose * actualPose.inverse();
        Matrix3 R = T.block(0, 0, 3, 3);
        complex.real((R.trace() - 1) / 2);
        double theta = acos(complex).real();

        // Algebra::round(theta);
        Vector3 w;
        if (theta <= 0.001 && theta >= -0.001)
        {
            w << 0, 0, 0;
        }
        else
        {
            w = (theta / (2.0 * sin(theta))) * Algebra::veeOperator(R - R.transpose());
        }

        Vector3 v1, v2, v;
        v1 << desiredPose(0, 3), desiredPose(1, 3), desiredPose(2, 3);
        v2 << actualPose(0, 3), actualPose(1, 3), actualPose(2, 3);
        v = v1 - v2;

        return (Vector6() << v(0), v(1), v(2), w(0), w(1), w(2)).finished();
    }

public:
    /// @brief Direct kinematic, return position and orientation of end effector from the joint angles
    /// @param angles joints angles
    static Matrix4 directKinematic(Vector6 angles)
    {
        Matrix4 res = Algebra::T01(angles(0)) * Algebra::T12(angles(1)) * Algebra::T23(angles(2)) *
                      Algebra::T34(angles(3)) * Algebra::T45(angles(4)) * Algebra::T56(angles(5));
        Algebra::round(res);
        return res;
    }
    /// @brief Inverse kinematic, return joint angles from end effector position and orientation 
    /// @param des_position_usd desire position of end effector
    /// @param des_orientation_usd desire orientation of end effector
    /// @param refAngles initial joints angles
    static Vector6 inverseKinematic(const Vector3 &des_position_usd, const Matrix3 &des_orientation_usd, const Vector6 &refAngles)
    {

        std::complex<double> complex(0, 0);
        Vector3 xHat;
        Vector3 yHat;

        IKMatrix th;
        th.setZero();

        Matrix4 des_pose_upsidedown;
        des_pose_upsidedown << des_orientation_usd(0, 0), des_orientation_usd(0, 1), des_orientation_usd(0, 2), des_position_usd(0),
            des_orientation_usd(1, 0), des_orientation_usd(1, 1), des_orientation_usd(1, 2), des_position_usd(1),
            des_orientation_usd(2, 0), des_orientation_usd(2, 1), des_orientation_usd(2, 2), des_position_usd(2),
            0, 0, 0, 1;

        Matrix4 transFrameToNormal;
        transFrameToNormal << 1, 0, 0, 0,
            0, cos(Constants::alph(0)), -sin(Constants::alph(0)), 0,
            0, sin(Constants::alph(0)), cos(Constants::alph(0)), 0,
            0, 0, 0, 1;

        Matrix4 desPos = transFrameToNormal * des_pose_upsidedown;
        Algebra::round(desPos);

        Vector3 pDes;
        pDes << desPos(0, 3), desPos(1, 3), desPos(2, 3); // get position x y z from Matrix transformed4

        // * ------------- theta1 -------------

        Vector4 p05;
        p05 = desPos * (Vector4() << 0, 0, -Constants::d(5), 1).finished();
        Algebra::round(p05);

        complex.real(Constants::d(3) / hypot(p05(1), p05(0)));
        double th11 = atan2(p05(1), p05[0]) + acos(complex).real() + (M_PI / 2);
        double th12 = atan2(p05(1), p05[0]) - acos(complex).real() + (M_PI / 2);
        round(th11);
        round(th12);

        // * ------------- theta5 -------------
        complex.real((pDes(0) * sin(th11) - pDes(1) * cos(th11) - Constants::d(3)) / Constants::d(5));
        double th51 = acos(complex).real();
        double th52 = -acos(complex).real();
        round(th51);
        round(th52);

        complex.real((pDes(0) * sin(th12) - pDes(1) * cos(th12) - Constants::d(3)) / Constants::d(5));
        double th53 = acos(complex).real();
        double th54 = -acos(complex).real();
        round(th53);
        round(th54);

        // * ------------- theta6 -------------
        Matrix4 T60 = desPos.inverse();
        Algebra::round(T60);
        xHat << T60(0, 0), T60(1, 0), T60(2, 0);
        yHat << T60(0, 1), T60(1, 1), T60(2, 1);
        Algebra::round(xHat);
        Algebra::round(yHat);

        double th61 = 0.0;
        double th62 = 0.0;
        double th63 = 0.0;
        double th64 = 0.0;

        if (th51 != 0)
        {
            th61 = atan2((-xHat(1) * sin(th11) + yHat(1) * cos(th11)) / sin(th51), (xHat(0) * sin(th11) - yHat(0) * cos(th11)) / sin(th51));
            round(th61);
        }
        if (th52 != 0)
        {
            th62 = atan2((-xHat(1) * sin(th11) + yHat(1) * cos(th11)) / sin(th52), (xHat(0) * sin(th11) - yHat(0) * cos(th11)) / sin(th52));
            round(th62);
        }
        if (th53 != 0)
        {
            th63 = atan2((-xHat(1) * sin(th12) + yHat(1) * cos(th12)) / sin(th53), (xHat(0) * sin(th12) - yHat(0) * cos(th12)) / sin(th53));
            round(th63);
        }
        if (th54 != 0)
        {
            th64 = atan2((-xHat(1) * sin(th12) + yHat(1) * cos(th12)) / sin(th54), (xHat(0) * sin(th12) - yHat(0) * cos(th12)) / sin(th54));
            round(th64);
        }

        // * ------------- theta3 -------------
        Matrix4 T14;
        T14 = Algebra::R01(th11).inverse() * desPos * Algebra::R56(th61).inverse() * Algebra::R45(th51).inverse();
        Algebra::round(T14);
        Vector3 p411;
        p411 << T14(0, 3), T14(1, 3), T14(2, 3);
        double p41xz1 = hypot(p411(0), p411(2));
        round(p41xz1);

        T14 = Algebra::R01(th11).inverse() * desPos * Algebra::R56(th62).inverse() * Algebra::R45(th52).inverse();
        Algebra::round(T14);
        Vector3 p412;
        p412 << T14(0, 3), T14(1, 3), T14(2, 3);
        double p41xz2 = hypot(p412(0), p412(2));
        round(p41xz2);

        T14 = Algebra::R01(th12).inverse() * desPos * Algebra::R56(th63).inverse() * Algebra::R45(th53).inverse();
        Algebra::round(T14);
        Vector3 p413;
        p413 << T14(0, 3), T14(1, 3), T14(2, 3);
        double p41xz3 = hypot(p413(0), p413(2));
        round(p41xz3);

        T14 = Algebra::R01(th12).inverse() * desPos * Algebra::R56(th64).inverse() * Algebra::R45(th54).inverse();
        Algebra::round(T14);
        Vector3 p414;
        p414 << T14(0, 3), T14(1, 3), T14(2, 3);
        double p41xz4 = hypot(p414(0), p414(2));
        round(p41xz4);

        complex.real((p41xz1 * p41xz1 - Constants::a(2) * Constants::a(2) - Constants::a(3) * Constants::a(3)) / (2 * Constants::a(2) * Constants::a(3)));
        double th31 = acos(complex).real();
        complex.real((p41xz2 * p41xz2 - Constants::a(2) * Constants::a(2) - Constants::a(3) * Constants::a(3)) / (2 * Constants::a(2) * Constants::a(3)));
        double th32 = acos(complex).real();
        complex.real((p41xz3 * p41xz3 - Constants::a(2) * Constants::a(2) - Constants::a(3) * Constants::a(3)) / (2 * Constants::a(2) * Constants::a(3)));
        double th33 = acos(complex).real();
        complex.real((p41xz4 * p41xz4 - Constants::a(2) * Constants::a(2) - Constants::a(3) * Constants::a(3)) / (2 * Constants::a(2) * Constants::a(3)));
        double th34 = acos(complex).real();

        double th35 = -th31;
        double th36 = -th32;
        double th37 = -th33;
        double th38 = -th34;

        round(th31);
        round(th32);
        round(th33);
        round(th34);
        round(th35);
        round(th36);
        round(th37);
        round(th38);

        // * ------------- theta2 -------------
        complex.real((-Constants::a(3) * sin(th31)) / p41xz1);
        double th21 = -abs(atan2(-p411(2), -p411(0)) - asin(complex).real());
        complex.real((-Constants::a(3) * sin(th32)) / p41xz2);
        double th22 = -abs(atan2(-p412(2), -p412(0)) - asin(complex).real());
        complex.real((-Constants::a(3) * sin(th33)) / p41xz3);
        double th23 = -abs(atan2(-p413(2), -p413(0)) - asin(complex).real());
        complex.real((-Constants::a(3) * sin(th34)) / p41xz4);
        double th24 = -abs(atan2(-p414(2), -p414(0)) - asin(complex).real());

        complex.real((Constants::a(3) * sin(th31)) / p41xz1);
        double th25 = -abs(atan2(-p411(2), -p411(0)) - asin(complex).real());
        complex.real((Constants::a(3) * sin(th32)) / p41xz2);
        double th26 = -abs(atan2(-p412(2), -p412(0)) - asin(complex).real());
        complex.real((Constants::a(3) * sin(th33)) / p41xz3);
        double th27 = -abs(atan2(-p413(2), -p413(0)) - asin(complex).real());
        complex.real((Constants::a(3) * sin(th34)) / p41xz4);
        double th28 = -abs(atan2(-p414(2), -p414(0)) - asin(complex).real());

        round(th21);
        round(th22);
        round(th23);
        round(th24);
        round(th25);
        round(th26);
        round(th27);
        round(th28);

        // * ------------- theta4 -------------
        Matrix4 T34;
        T34 = Algebra::R23(th31).inverse() * Algebra::R12(th21).inverse() * Algebra::R01(th11).inverse() * desPos * Algebra::R56(th61).inverse() * Algebra::R45(th51).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th41 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th32).inverse() * Algebra::R12(th22).inverse() * Algebra::R01(th11).inverse() * desPos * Algebra::R56(th62).inverse() * Algebra::R45(th52).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th42 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th33).inverse() * Algebra::R12(th23).inverse() * Algebra::R01(th12).inverse() * desPos * Algebra::R56(th63).inverse() * Algebra::R45(th53).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th43 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th34).inverse() * Algebra::R12(th24).inverse() * Algebra::R01(th12).inverse() * desPos * Algebra::R56(th64).inverse() * Algebra::R45(th54).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th44 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th35).inverse() * Algebra::R12(th25).inverse() * Algebra::R01(th11).inverse() * desPos * Algebra::R56(th61).inverse() * Algebra::R45(th51).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th45 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th36).inverse() * Algebra::R12(th26).inverse() * Algebra::R01(th11).inverse() * desPos * Algebra::R56(th62).inverse() * Algebra::R45(th52).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th46 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th37).inverse() * Algebra::R12(th27).inverse() * Algebra::R01(th12).inverse() * desPos * Algebra::R56(th63).inverse() * Algebra::R45(th53).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th47 = atan2(xHat(1), xHat(0));

        T34 = Algebra::R23(th38).inverse() * Algebra::R12(th28).inverse() * Algebra::R01(th12).inverse() * desPos * Algebra::R56(th64).inverse() * Algebra::R45(th54).inverse();
        Algebra::round(T34);
        xHat << T34(0, 0), T34(1, 0), T34(2, 0);
        double th48 = atan2(xHat(1), xHat(0));

        round(th41);
        round(th42);
        round(th43);
        round(th44);
        round(th45);
        round(th46);
        round(th47);
        round(th48);

        th << th11, th11, th12, th12, th11, th11, th12, th12,
            th21, th22, th23, th24, th25, th26, th27, th28,
            th31, th32, th33, th34, th35, th36, th37, th38,
            th41, th42, th43, th44, th45, th46, th47, th48,
            th51, th52, th53, th54, th51, th52, th53, th54,
            th61, th62, th63, th64, th61, th62, th63, th64;

        double actualNorm = (refAngles - th.col(0)).norm();
        double norm;
        double posError;
        Vector6 bestAngles = th.col(0);
        Vector3 eePos;
        Matrix4 temp;

        for (int i = 1; i < 8; i++)
        {
            norm = (refAngles - th.col(i)).norm();

            temp = directKinematic(th.col(i));
            eePos << temp(0, 3), temp(1, 3), temp(2, 3);
            posError = (eePos - des_position_usd).norm();

            if (norm <= actualNorm && posError < posErrorBoundary)
            {
                actualNorm = norm;
                bestAngles = th.col(i);
            }
        }

        return bestAngles;
    }

    static std::vector<std::vector<double>> inversDifferentialKinematic(const Vector6 &jstate, const Matrix4 &desiredPose, const double precision, const double damping = 0.04, const double maxDelta = max_dq, const double timeScale = 1)
    {
        std::vector<std::vector<double>> positions;

        Vector6 newJstate, nextJstate;
        newJstate = Vector6(jstate);
        nextJstate = Vector6(jstate);
        positions.push_back(Algebra::convert(nextJstate));

        Matrix4 actualPose = directKinematic(jstate);
        double delta = Matrix4(actualPose - desiredPose).norm();
        Vector6 step, dtheta, scaledDtheta;
        Matrix6 J;
        Matrix4 previousPose;
        double minSingVal, scaledelta;
        while (delta > precision)
        {
            newJstate = nextJstate;
            step = getDirectionStep(actualPose, desiredPose);
            J = Algebra::jacobian(newJstate);
            Eigen::JacobiSVD<Matrix6> svd(J);
            minSingVal = svd.singularValues().minCoeff();

            if (minSingVal < 0.001)
            {
                dtheta = J.transpose() * (J * J.transpose() + (damping * damping * Matrix6().setIdentity())).inverse() * step;
                std::cout << "Correzione di singolarità" << std::endl;
            }
            else
            {
                dtheta = J.inverse() * step;
            }

            scaledelta = maxDelta / std::max(maxDelta, dtheta.cwiseAbs().maxCoeff());
            scaledDtheta = timeScale * scaledelta * dtheta;
            nextJstate = newJstate + scaledDtheta;
            positions.push_back(Algebra::convert(nextJstate));
            previousPose = Matrix4(actualPose);
            actualPose = directKinematic(nextJstate);
            delta = (actualPose - previousPose).norm();
        }
        positions.pop_back();
        return positions;
    }

    static Trajectory differentialKinematic(const Vector6 &initial_jstate, const Vector3 &final_p, const Matrix3 &final_rotm, const CurveType &curveType = CurveType::BEZIER, const double &vel = 1)
    {
        Trajectory trajectory;
        Matrix4 intial_pose = directKinematic(initial_jstate);
        Vector3 initial_p;
        initial_p << intial_pose(0, 3), intial_pose(1, 3), intial_pose(2, 3);
        Matrix3 initial_rotm = intial_pose.block(0, 0, 3, 3);

        float ds = 0.05;
        std::vector<Vector3> path;
        std::vector<Matrix3> rotms;

        // rotms = Algebra::matrixLinspace(initial_rotm, final_rotm, (int)(1 / ds));
        rotms = Algebra::slerp(initial_rotm, final_rotm, (int)(1/ds));

        if (curveType == CurveType::BEZIER)
        {
            path = Algebra::bezierPath(initial_p, final_p, (int)(1 / ds));
        }
        else if (curveType == CurveType::LINE)
        {
            path = Algebra::linePath(initial_p, final_p, (int)(1 / ds));
        }
        else
        {
            std::cout << "Curva non disponibile!" << std::endl;
            return trajectory;
        }

        Vector6 actual_jstate = initial_jstate;
        trajectory.positions.push_back(Algebra::convert(actual_jstate));
        Matrix4 nextPose;
        std::vector<std::vector<double>> newViaPoints;
        for (int i = 1; i < path.size(); i++)
        {
            nextPose << rotms[i](0, 0), rotms[i](0, 1), rotms[i](0, 2), path[i](0),
                rotms[i](1, 0), rotms[i](1, 1), rotms[i](1, 2), path[i](1),
                rotms[i](2, 0), rotms[i](2, 1), rotms[i](2, 2), path[i](2),
                0, 0, 0, 1;

            if (i < path.size() - 3)
            {
                newViaPoints = inversDifferentialKinematic(actual_jstate, nextPose, intermediatePrecision);
            }
            else
            {
                newViaPoints = inversDifferentialKinematic(actual_jstate, nextPose, finalPrecision);
            }

            if (newViaPoints.size() <= 1)
            {
                continue;
            }
            else
            {
                if (Algebra::norm(Algebra::diff(trajectory.positions.back(), newViaPoints.front())) < 0.0005)
                {
                    newViaPoints.erase(newViaPoints.begin());
                }
            }
            for (auto i : newViaPoints)
            {
                trajectory.positions.push_back(i);
            }
            std::vector<double> temp = newViaPoints.back();
            actual_jstate = (Vector6() << temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]).finished();
        }

        std::vector<double> dtheta;

        std::vector<std::vector<double>> temPosition;
        temPosition.push_back(trajectory.positions[0]);
        for (int i = 1; i < trajectory.positions.size(); i++)
        {
            dtheta = Algebra::diff(trajectory.positions[i], trajectory.positions[i - 1]);
            if (Algebra::max(dtheta) > 0.0005)
            {
                temPosition.push_back(trajectory.positions[i]);
            }
        }
        trajectory.positions = temPosition;

        trajectory.times.push_back(0.0);
        // trajectory.velocities.push_back({vel, vel, vel, vel, vel, vel});
        for (int i = 1; i < trajectory.positions.size(); i++)
        {
            dtheta = Algebra::diff(trajectory.positions[i], trajectory.positions[i - 1]);
            trajectory.times.push_back(trajectory.times[i - 1] + Algebra::max(Algebra::abs(dtheta)) / vel);
            // trajectory.velocities.push_back({vel, vel, vel, vel, vel, vel});
        }

        return trajectory;
    }

    /// @brief create a trajectory in the joints space with a cubic interpolation
    /// @param qi initial joints positions
    /// @param qf final joints positions
    /// @param vi initial velocities
    /// @param vf final velocities 
    /// @param vmax max velocities
    static Trajectory jcubic(Vector6 qi, Vector6 qf, Vector6 vi = (Vector6() << 0, 0, 0, 0, 0, 0).finished(), Vector6 vf = (Vector6() << 0, 0, 0, 0, 0, 0).finished(), double vmax = 1.0)
    {
        Trajectory trajectory;
        Vector6 h = qf - qi;
        double dqMax = Algebra::abs(h).maxCoeff();
        double T = dqMax / vmax;
        int n = (int) (dqMax / max_dq);
        std::cout << "jcubic: T=" << T << ", n=" << n << std::endl;
        Matrix2 Tm;
        Tm << T * T, T * T * T, 2 * T, 3 * T * T;
        Matrix2 Tinv = Tm.inverse();
        Vector6 a2 = Tinv(0, 0) * (h - vi * T) + Tinv(0, 1) * (vf - vi);
        Vector6 a3 = Tinv(1, 0) * (h - vi * T) + Tinv(1, 1) * (vf - vi);

        trajectory.times = Algebra::linspace(0, T, n);

        for (double t : trajectory.times)
        {
            trajectory.positions.push_back(Algebra::convert(qi + vi*t + a2*pow(t,2) + a3*pow(t,3)));
            trajectory.velocities.push_back(Algebra::convert(vi + 2*a2*t + 3*a3*pow(t,2)));
        }

        return trajectory;
    }
};

#endif
