#ifndef __ALGEBRA_H__
#define __ALGEBRA_H__
#include "constants.h"

#define ROUND_TRESHOLD 0.0001
#define DECIMAL_PRECISION 100000.0
#define DIFF_DT 0.005
#define DIFF_TMIN 0.0

class Algebra
{
public:
    static Matrix3 rotY(const double &theta)
    {
        return (Matrix3() << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta))
            .finished();
    }

    static Matrix4 T01(const double &theta1)
    {
        Matrix4 rotX;
        rotX << 1, 0, 0, 0,
            0, cos(Constants::alph(0)), -sin(Constants::alph(0)), 0,
            0, sin(Constants::alph(0)), cos(Constants::alph(0)), 0,
            0, 0, 0, 1;

        Matrix4 traslZ;
        traslZ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, Constants::d(0),
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta1), -sin(theta1), 0, 0,
            sin(theta1), cos(theta1), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = rotX * traslZ * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T12(const double &theta2)
    {
        Matrix4 rotX;
        rotX << 1, 0, 0, 0,
            0, cos(Constants::alph(1)), -sin(Constants::alph(1)), 0,
            0, sin(Constants::alph(1)), cos(Constants::alph(1)), 0,
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta2), -sin(theta2), 0, 0,
            sin(theta2), cos(theta2), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = rotX * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T23(const double &theta3)
    {
        Matrix4 traslX;
        traslX << 1, 0, 0, Constants::a(2),
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta3), -sin(theta3), 0, 0,
            sin(theta3), cos(theta3), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = traslX * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T34(const double &theta4)
    {
        Matrix4 traslXZ;
        traslXZ << 1, 0, 0, Constants::a(3),
            0, 1, 0, 0,
            0, 0, 1, Constants::d(3),
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta4), -sin(theta4), 0, 0,
            sin(theta4), cos(theta4), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = traslXZ * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T45(const double &theta5)
    {
        Matrix4 rotX;
        rotX << 1, 0, 0, 0,
            0, cos(Constants::alph(4)), -sin(Constants::alph(4)), 0,
            0, sin(Constants::alph(4)), cos(Constants::alph(4)), 0,
            0, 0, 0, 1;

        Matrix4 traslZ;
        traslZ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, Constants::d(4),
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta5), -sin(theta5), 0, 0,
            sin(theta5), cos(theta5), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = rotX * traslZ * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T56(const double &theta6)
    {
        Matrix4 rotX;
        rotX << 1, 0, 0, 0,
            0, cos(Constants::alph(5)), -sin(Constants::alph(5)), 0,
            0, sin(Constants::alph(5)), cos(Constants::alph(5)), 0,
            0, 0, 0, 1;

        Matrix4 traslZ;
        traslZ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, Constants::d(5),
            0, 0, 0, 1;

        Matrix4 rotZ;
        rotZ << cos(theta6), -sin(theta6), 0, 0,
            sin(theta6), cos(theta6), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Matrix4 res = rotX * traslZ * rotZ;
        round(res);
        return res;
    }
    static Matrix4 T06()
    {
        Matrix4 res = T01(0) * T12(0) * T23(0) * T34(0) * T45(0) * T56(0);
        round(res);
        return res;
    }

    static Matrix4 R01(const double &theta1)
    {
        Matrix4 res = (Matrix4() << cos(theta1), -sin(theta1), 0, 0,
                       sin(theta1), cos(theta1), 0, 0,
                       0, 0, 1, Constants::d(0),
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }
    static Matrix4 R12(const double &theta2)
    {
        Matrix4 res = (Matrix4() << cos(theta2), -sin(theta2), 0, 0,
                       0, 0, -1, 0,
                       sin(theta2), cos(theta2), 0, 0,
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }
    static Matrix4 R23(const double &theta3)
    {
        Matrix4 res = (Matrix4() << cos(theta3), -sin(theta3), 0, Constants::a(2),
                       sin(theta3), cos(theta3), 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }
    static Matrix4 R34(const double &theta4)
    {
        Matrix4 res = (Matrix4() << cos(theta4), -sin(theta4), 0, Constants::a(3),
                       sin(theta4), cos(theta4), 0, 0,
                       0, 0, 1, Constants::d(3),
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }
    static Matrix4 R45(const double &theta5)
    {
        Matrix4 res = (Matrix4() << cos(theta5), -sin(theta5), 0, 0,
                       0, 0, -1, -Constants::d(4),
                       sin(theta5), cos(theta5), 0, 0,
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }
    static Matrix4 R56(const double &theta6)
    {
        Matrix4 res = (Matrix4() << cos(theta6), -sin(theta6), 0, 0,
                       0, 0, 1, Constants::d(5),
                       -sin(theta6), -cos(theta6), 0, 0,
                       0, 0, 0, 1)
                          .finished();
        round(res);
        return res;
    }

    static Matrix2 &round(Matrix2 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Matrix3 &round(Matrix3 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Matrix4 &round(Matrix4 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Matrix6 &round(Matrix6 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static IKMatrix &round(IKMatrix &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Vector3 &round(Vector3 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Vector4 &round(Vector4 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static Vector6 &round(Vector6 &mat)
    {
        mat = mat.unaryExpr([](double elem)
                            { double x = elem = std::round(elem * DECIMAL_PRECISION) / DECIMAL_PRECISION; 
                                
                                if(std::abs(x) < ROUND_TRESHOLD)
                                {
                                    x = 0.0000;
                                }
                                return x; });
        return mat;
    }
    static double &round(double &mat)
    {
        mat = std::round(mat * DECIMAL_PRECISION) / DECIMAL_PRECISION;

        if (std::abs(mat) < ROUND_TRESHOLD)
        {
            mat = 0.0;
        }
        return mat;
    }

    static int factorial(int n)
    {
        return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
    }
    static std::vector<double> linspace(const double &start, const double &end, const int &num)
    {

        std::vector<double> linspaced;

        if (num == 0)
        {
            return linspaced;
        }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for (int i = 0; i < num - 1; ++i)
        {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end); // I want to ensure that start and end

        return linspaced;
    }
    static std::vector<Vector3> linspace(const Vector3 &start, const Vector3 &end, const int &num)
    {
        std::vector<Vector3> linspaced;

        if (num == 0)
        {
            return linspaced;
        }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }

        Vector3 delta = (end - start) / (num - 1);

        for (int i = 0; i < num - 1; ++i)
        {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end); // I want to ensure that start and end

        return linspaced;
    }
    static std::vector<Matrix3> matrixLinspace(const Matrix3 &rotm_i, const Matrix3 &rotm_f, int n)
    {
        std::vector<Vector3> x_dir, y_dir, z_dir;
        std::vector<Matrix3> mlinspace;
        x_dir = linspace((Vector3)rotm_i.block(0, 0, 3, 1), (Vector3)rotm_f.block(0, 0, 3, 1), n);
        y_dir = linspace((Vector3)rotm_i.block(0, 1, 3, 1), (Vector3)rotm_f.block(0, 1, 3, 1), n);
        z_dir = linspace((Vector3)rotm_i.block(0, 2, 3, 1), (Vector3)rotm_f.block(0, 2, 3, 1), n);

        for (int i = 0; i < n; i++)
        {
            mlinspace.push_back(
                (Matrix3() << x_dir[i](0), y_dir[i](0), z_dir[i](0),
                 x_dir[i](1), y_dir[i](1), z_dir[i](1),
                 x_dir[i](2), y_dir[i](2), z_dir[i](2))
                    .finished());
        }
        return mlinspace;
    }
    static std::vector<Matrix3> slerp(const Matrix3 &rotm_i, const Matrix3 &rotm_f, int n) {
        Vector4 q_i, q_f, q_m;
        q_i = rotm2quat(rotm_i);
        q_f = rotm2quat(rotm_f);

        std::vector<double> ls = linspace(0.0, 1.0, n);
        std::vector<Matrix3> slerp;
        Matrix3 rotm_m;
        std::complex<double> complex(0, 0);
        
        complex.real(q_i(0)*q_f(0) + q_i(1)*q_f(1) + q_i(2)*q_f(2) + q_i(3)*q_f(3));

        if (complex.real() < 0)
        {
            q_f(0)=-q_f(0); q_f(1)=-q_f(1); q_f(2)=-q_f(2); q_f(3)=-q_f(3);
            complex.real(-complex.real());
        }

        double angle = acos(complex).real();
        double denom = sin(angle);

        for (double t : ls)
        {
            if (round(denom) != 0.0)
            {
                q_m = (q_i*sin((1-t)*angle) + q_f*sin(t*angle)) / denom;
                rotm_m = quat2rotm(q_m);
            }
            else
                rotm_m = rotm_f;
            
            slerp.push_back(rotm_m);
        }
        return slerp;
    }

    static int binomialCoeff(int n, int k)
    {
        return factorial(n) / (factorial(k) * factorial(n - k));
    }

    static Matrix6 jacobian(const Vector6 &th)
    {
        // Jacobiana dell'ur5 in posizione "a testa in giù", derivata dalla DK customizzata
        // asse z0 e origine frame 0
        Vector3 z0, p0;
        z0 << 0, 0, 1;
        p0 << 0, 0, 0;

        // asse z1 e origine frame 1
        Matrix4 t01 = T01(th(0));
        round(t01);
        Vector3 z1;
        Vector3 p1;
        z1 << t01(0, 2), t01(1, 2), t01(2, 2);
        p1 << t01(0, 3), t01(1, 3), t01(2, 3);

        // asse z2 e origine frame 2
        Matrix4 t02 = t01 * T12(th(1));
        round(t02);
        Vector3 z2;
        Vector3 p2;
        z2 << t02(0, 2), t02(1, 2), t02(2, 2);
        p2 << t02(0, 3), t02(1, 3), t02(2, 3);

        // asse z3 e origine frame 3
        Matrix4 t03 = t02 * T23(th(2));
        round(t03);
        Vector3 z3;
        Vector3 p3;
        z3 << t03(0, 2), t03(1, 2), t03(2, 2);
        p3 << t03(0, 3), t03(1, 3), t03(2, 3);

        // asse z4 e origine frame 4
        Matrix4 t04 = t03 * T34(th(3));
        round(t04);
        Vector3 z4;
        Vector3 p4;
        z4 << t04(0, 2), t04(1, 2), t04(2, 2);
        p4 << t04(0, 3), t04(1, 3), t04(2, 3);

        // asse z5 e origine frame 5
        Matrix4 t05 = t04 * T45(th(4));
        round(t05);
        Vector3 z5;
        Vector3 p5;
        z5 << t05(0, 2), t05(1, 2), t05(2, 2);
        p5 << t05(0, 3), t05(1, 3), t05(2, 3);

        // asse z6 e origine frame 6
        Matrix4 t06 = t05 * T56(th(5));
        // ! questa matrice ha un solo valore leggermente errato
        round(t06);
        Vector3 z6;
        Vector3 p6;
        z6 << t06(0, 2), t06(1, 2), t06(2, 2);
        p6 << t06(0, 3), t06(1, 3), t06(2, 3);

        Matrix6 J;
        Vector3 z1xp = z1.cross(p6 - p1);
        Vector3 z2xp = z2.cross(p6 - p2);
        Vector3 z3xp = z3.cross(p6 - p3);
        Vector3 z4xp = z4.cross(p6 - p4);
        Vector3 z5xp = z5.cross(p6 - p5);
        Vector3 z6xp = z6.cross(p6 - p6);
        J << z1xp(0), z2xp(0), z3xp(0), z4xp(0), z5xp(0), z6xp(0),
            z1xp(1), z2xp(1), z3xp(1), z4xp(1), z5xp(1), z6xp(1),
            z1xp(2), z2xp(2), z3xp(2), z4xp(2), z5xp(2), z6xp(2),
            z1(0), z2(0), z3(0), z4(0), z5(0), z6(0),
            z1(1), z2(1), z3(1), z4(1), z5(1), z6(1),
            z1(2), z2(2), z3(2), z4(2), z5(2), z6(2);

        round(J);

        return J;
    }

    static Matrix3 eul2RotM(const Vector3 &euler_angles)
    {
        Matrix3 rotx;
        rotx << 1, 0, 0,
            0, cos(euler_angles(0)), -sin(euler_angles(0)),
            0, sin(euler_angles(0)), cos(euler_angles(0));
        Matrix3 roty;
        roty << cos(euler_angles(1)), 0, sin(euler_angles(1)),
            0, 1, 0,
            -sin(euler_angles(1)), 0, cos(euler_angles(1));
        Matrix3 rotz;
        rotz << cos(euler_angles(2)), -sin(euler_angles(2)), 0,
            sin(euler_angles(2)), cos(euler_angles(2)), 0,
            0, 0, 1;

        Matrix3 res = rotz * roty * rotx;
        round(res);
        return res;
    }
    static Vector4 rotm2quat(const Matrix3 &m)
    {
        double tr = m(0,0) + m(1,1) + m(2,2), S, qw, qx, qy, qz;

        if (tr > 0)
        {
            S = sqrt(tr+1.0) * 2;  // S=4*qw 
            qw = 0.25 * S;
            qx = (m(2,1) - m(1,2)) / S;
            qy = (m(0,2) - m(2,0)) / S;
            qz = (m(1,0) - m(0,1)) / S;
        }
        else if ((m(0,0) > m(1,1)) && (m(0,0) > m(2,2)))
        {
            S = sqrt(1.0 + m(0,0) - m(1,1) - m(2,2)) * 2;  // S=4*qx 
            qw = (m(2,1) - m(1,2)) / S;
            qx = 0.25 * S;
            qy = (m(0,1) + m(1,0)) / S;
            qz = (m(0,2) + m(2,0)) / S;
        }
        else if (m(1,1) > m(2,2))
        {
            S = sqrt(1.0 + m(1,1) - m(0,0) - m(2,2)) * 2;  // S=4*qy
            qw = (m(0,2) - m(2,0)) / S;
            qx = (m(0,1) + m(1,0)) / S;
            qy = 0.25 * S;
            qz = (m(1,2) + m(2,1)) / S;
        }
        else
        {
            S = sqrt(1.0 + m(2,2) - m(0,0) - m(1,1)) * 2;  // S=4*qz
            qw = (m(1,0) - m(0,1)) / S;
            qx = (m(0,2) + m(2,0)) / S;
            qy = (m(1,2) + m(2,1)) / S;
            qz = 0.25 * S;
        }

        return (Vector4() << qw, qx, qy, qz).finished();
    }
    static Matrix3 quat2rotm(const Vector4 &q)
    {
        return (Matrix3() << 
                q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3), 2*(q(1)*q(2) - q(0)*q(3)),                     2*(q(1)*q(3) + q(0)*q(2)),
                2*(q(1)*q(2) + q(0)*q(3)),                     q(0)*q(0) - q(1)*q(1) + q(2)*q(2) - q(3)*q(3), 2*(q(2)*q(3) - q(0)*q(1)),
                2*(q(1)*q(3) - q(0)*q(2)),                     2*(q(2)*q(3) + q(0)*q(1)),                     q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)
                ).finished();
    }
    static Vector3 rotM2Eul(const Matrix3 &r)
    {
        double sy = sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
        double theta1, theta2, csi1, csi2, phi1, phi2;

        if (r(2, 0) != 1 && r(2, 0) != -1)
        {
            theta1 = -asin(std::complex<double>(r(2, 0), 0)).real();
            theta2 = M_PI - theta1;

            csi1 = atan2(r(2, 1) / cos(theta1), r(2, 2) / cos(theta1));
            csi2 = atan2(r(2, 1) / cos(theta2), r(2, 2) / cos(theta2));

            phi1 = atan2(r(1, 0) / cos(theta1), r(0, 0) / cos(theta1));
            phi2 = atan2(r(1, 0) / cos(theta2), r(0, 0) / cos(theta2));
        }
        else
        {
            phi1 = 0.0;
            if (r(2, 0) == -1)
            {
                theta1 = M_PI / 2;
                csi1 = phi1 + atan2(r(0, 1), r(0, 2));
            }
            else
            {
                theta1 = -M_PI / 2;
                csi1 = -phi1 + atan2(-r(0, 1), -r(0, 2));
            }
        }

        return (Vector3() << csi1, theta1, phi1).finished();
    }
    static Vector3 veeOperator(Matrix3 mat)
    {
        return (Vector3() << mat(2, 1), mat(0, 2), mat(1, 0)).finished();
    }

    static Vector3 bezierEquation(const std::vector<Vector3> &p, int n, double t)
    {
        Vector3 b = Vector3().setZero();
        int bin;
        for (int k = 0; k <= n; k++)
        {
            bin = Algebra::binomialCoeff(n, k);
            b += bin * pow(1 - t, n - k) * pow(t, k) * p[k];
        }

        return b;
    }
    static std::vector<Vector3> bezierPath(const Vector3 &start_p, const Vector3 &end_p, int n)
    {

        std::vector<double> T = Algebra::linspace(0, 1, n);

        std::vector<Vector3> points;
        Vector3 mid_p, mid2_p, mid3_p, mid4_p;
        mid_p << start_p(0) + (end_p(0) - start_p(0)) / 10, std::max(start_p(1), end_p(1)) - 1.43 * std::max(start_p(1), end_p(1)) + 1, start_p(2) + (end_p(2) - start_p(2)) / 2;
        mid4_p << start_p(0) + (end_p(0) - start_p(0)) * 9 / 10, std::max(start_p(1), end_p(1)) - 1.43 * std::max(start_p(1), end_p(1)) + 1, start_p(2) + (end_p(2) - start_p(2)) / 2;
        mid2_p << end_p(0), end_p(1) - 0.1, end_p(2) + 0.3;
        mid3_p << start_p(0), start_p(1), start_p(2) + 0.25;

        std::vector<Vector3> p = {start_p, mid3_p, mid_p, mid4_p, mid2_p, end_p};

        for (double t : T)
        {
            points.push_back(bezierEquation(p, 5, t));
        }

        return points;
    }
    static std::vector<Vector3> linePath(const Vector3 &start_p, const Vector3 &end_p, int n)
    {
        return linspace(start_p, end_p, n);
    };

    static std::vector<double> convert(const Vector6 &vec)
    {
        std::vector<double> res;
        for (int i = 0; i < vec.size(); i++)
        {
            res.push_back(vec(i));
        }
        return res;
    }
    static std::vector<double> diff(const std::vector<double> &v1, const std::vector<double> &v2)
    {
        std::vector<double> res;
        for (int i = 0; i < v1.size(); i++)
        {
            res.push_back(v1[i] - v2[i]);
        }
        return res;
    }
    static double norm(const std::vector<double> &v)
    {
        double res;
        for (int i = 0; i < v.size(); i++)
        {
            res += i * i;
        }
        return sqrt(res);
    }
    static double max(const std::vector<double> &v)
    {
        double res = v[0];
        for (int i = 1; i < v.size(); i++)
        {
            if (res < v[i])
            {
                res = v[i];
            }
        }
        return res;
    }
    static double min(const std::vector<double> &v)
    {
        double res = v[0];
        for (int i = 1; i < v.size(); i++)
        {
            if (res > v[i])
            {
                res = v[i];
            }
        }
        return res;
    }
    static std::vector<double> abs(const std::vector<double> &v)
    {
        std::vector<double> res;
        for (int i = 0; i < v.size(); i++)
        {
            res.push_back(std::abs(v[i]));
        }
        return res;
    }
    static Vector6 abs(const Vector6 &v)
    {
        return (Vector6()<< std::abs(v(0)),std::abs(v(1)),std::abs(v(2)),std::abs(v(3)),std::abs(v(4)),std::abs(v(5))).finished();
    }

    template <typename T>
    static int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
};

/*
 * Cartesian coordinates
 */
struct Cartesian
{
    Cartesian()
    {
        x = 0;
        y = 0;
        z = 0;
    };
    Cartesian(double _x, double _y, double _z)
    {
        x = _x;
        y = _y;
        z = _z;
    };
    double x;
    double y;
    double z;
    Cartesian operator-(Cartesian car)
    {
        return Cartesian(x - car.x, y - car.y, z - car.z);
    }
    Cartesian operator+(Cartesian car)
    {
        return Cartesian(x + car.x, y + car.y, z + car.z);
    }
    Vector3 toVector(){
        return (Vector3() << x,y,z).finished();
    }
};

std::ostream &operator<<(std::ostream &o, const Cartesian &a)
{
    return o << "[" << a.x << "," << a.y << "," << a.z << "]";
}

struct RPY
{
    RPY()
    {
        r = 0;
        p = 0;
        y = 0;
    };
    RPY(double _r, double _p, double _y)
    {
        r = _r;
        p = _p;
        y = _y;
    };
    double r;
    double p;
    double y;
    RPY operator-(RPY car)
    {
        return RPY(r - car.r, p - car.p, y - car.y);
    }
    RPY operator+(RPY car)
    {
        return RPY(r + car.r, p + car.p, y + car.y);
    }
    Vector3 toVector()
    {
        return (Vector3() << r, p, y).finished();
    }
    bool isNull()
    {
        if (r==-1 && p==-1 && y==-1)
            return true;
        else
            return false;
    }
};

struct Quaternion
{
    Quaternion(double _x, double _y, double _z, double _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    };
    double x;
    double y;
    double z;
    double w;

    RPY toRpy() const
    {
        double yaw;
        double temp = atan2(2 * (w * x + y * z), w * w - x * x - y * y + z * z);
        double roll = Algebra::round(temp);
        temp = asin(std::complex<double>(2 * (w * y - x * z), 0)).real();
        double pitch = Algebra::round(temp);
        if ((M_PI / 2 - 0.001 < pitch && pitch < M_PI / 2 + 0.001) || (-M_PI / 2 - 0.001 < pitch && pitch < -M_PI / 2 + 0.001))
        {
            roll = 0;
            temp = -2 * Algebra::sgn(pitch) * atan2(x, w);
            yaw = Algebra::round(temp);
        }
        else
        {
            temp = atan2(2 * (w * z + x * y), w * 2 + x * 2 - y * 2 - z * 2);
            yaw = Algebra::round(temp);
        }
        return RPY(roll, pitch, yaw);
    }
};

#endif
