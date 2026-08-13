/*******************************************************************************
 * @file process_model.hpp
 * @brief 过程模型定义，描述排球运动的物理模型（含空气阻力）
 *******************************************************************************/

#ifndef VOLLEYBALL_PROCESS_MODEL_HPP
#define VOLLEYBALL_PROCESS_MODEL_HPP

#include <functional>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace volleyball
{

/****************************************************************
 * @struct ProcessModel 过程模型（匿名函数容器）
 *        jacobian_f 可为空（UKF不需要）
 ****************************************************************/
struct ProcessModel
{
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> f;
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> Q;
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> jacobian_f;
};

/****************************************************************
 * @brief 公共 Q 矩阵：离散化白噪声模型
 ****************************************************************/
inline Eigen::MatrixXd make_white_noise_Q(double sx2, double sy2, double sz2, double dt)
{
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    Eigen::Matrix2d Q_block;
    Q_block << 0.25 * dt4, 0.5 * dt3,
               0.5  * dt3,       dt2;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    Q.block<2, 2>(0, 0) = Q_block * sx2;
    Q.block<2, 2>(2, 2) = Q_block * sy2;
    Q.block<2, 2>(4, 4) = Q_block * sz2;
    return Q;
}

// ===================================================================
//  工厂函数
// ===================================================================

/****************************************************************
 * @brief 三维自由落体模型（无空气阻力）
 ****************************************************************/
inline ProcessModel makeBallist3D(double q_sxy_min, double q_sxy_max,
                                  double q_sz_min, double q_sz_max, double g = 9.8)
{

    auto f = [g](const Eigen::VectorXd& x, double dt) -> Eigen::VectorXd
    {
        Eigen::MatrixXd F(6, 6);
        F << 1, dt, 0, 0, 0, 0,
             0, 1,  0, 0, 0, 0,
             0, 0,  1, dt,0, 0,
             0, 0,  0, 1, 0, 0,
             0, 0,  0, 0, 1, dt,
             0, 0,  0, 0, 0, 1;
        Eigen::VectorXd b(6);
        b << 0, 0, 0, 0, 0.5 * (-g) * dt * dt, (-g) * dt;
        return F * x + b;
    };

    auto jacobian_f = [](const Eigen::VectorXd&, double dt) -> Eigen::MatrixXd
    {
        Eigen::MatrixXd F(6, 6);
        F << 1, dt, 0, 0, 0, 0,
             0, 1,  0, 0, 0, 0,
             0, 0,  1, dt,0, 0,
             0, 0,  0, 1, 0, 0,
             0, 0,  0, 0, 1, dt,
             0, 0,  0, 0, 0, 1;
        return F;
    };

    double q_sxy2_max = std::pow(q_sxy_max, 2);
    double q_sxy2_min = std::pow(q_sxy_min, 2);
    double q_sz2_max = std::pow(q_sz_max, 2);
    double q_sz2_min = std::pow(q_sz_min, 2);
    auto Q = [q_sxy2_max, q_sxy2_min, q_sz2_max, q_sz2_min](const Eigen::VectorXd& x_post, double dt) -> Eigen::MatrixXd
    {
        double vx = x_post[1];
        double vy = x_post[3];
        double vz = x_post[5];
        double v = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2) + std::pow(vz, 2));

        double scale_xy = std::exp(-v) * (q_sxy2_max - q_sxy2_min) + q_sxy2_min;
        double scale_z = std::exp(-v) * (q_sz2_max - q_sz2_min) + q_sz2_min;

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        Eigen::Matrix2d Q_block;
        Q_block << 0.25 * dt4, 0.5 * dt3,
                    0.5  * dt3,       dt2;

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
        Q.block<2, 2>(0, 0) = Q_block * scale_xy;
        Q.block<2, 2>(2, 2) = Q_block * scale_xy;
        Q.block<2, 2>(4, 4) = Q_block * scale_z;

        return Q;
    };

    return { f, Q, jacobian_f };
}

/****************************************************************
 * @brief 三维线性空气阻力模型
 ****************************************************************/
inline ProcessModel makeLinearDrag3D(double q_sxy_min, double q_sxy_max,
                                     double q_sz_min, double q_sz_max,
                                     double k, double m, double g = 9.8)
{
    double q_sxy2_min = std::pow(q_sxy_min, 2);
    double q_sxy2_max = std::pow(q_sxy_max, 2);
    double q_sz2_min = std::pow(q_sz_min, 2);
    double q_sz2_max = std::pow(q_sz_max, 2);
    double beta = k / m;

    auto f = [beta, g](const Eigen::VectorXd& x, double dt) -> Eigen::VectorXd
    {
        double phi = std::exp(-beta * dt);
        double n   = (1.0 - phi) / beta;

        Eigen::Matrix2d F_block;
        F_block << 1, n,
                   0, phi;

        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        F.block<2, 2>(0, 0) = F_block;
        F.block<2, 2>(2, 2) = F_block;
        F.block<2, 2>(4, 4) = F_block;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        Eigen::VectorXd b(6);
        b << 0, 0,
             0, 0,
             (-g) * (dt - n) / beta,
             (-g) * n;

        return F * x + b;
    };

    auto jacobian_f = [beta](const Eigen::VectorXd&, double dt) -> Eigen::MatrixXd
    {
        double phi = std::exp(-beta * dt);
        double n   = (1.0 - phi) / beta;

        Eigen::Matrix2d F_block;
        F_block << 1, n,
                   0, phi;

        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        F.block<2, 2>(0, 0) = F_block;
        F.block<2, 2>(2, 2) = F_block;
        F.block<2, 2>(4, 4) = F_block;
        return F;
    };

    auto Q = [q_sxy2_min, q_sxy2_max, q_sz2_min, q_sz2_max](const Eigen::VectorXd& x, double dt) -> Eigen::MatrixXd
    {
        double vx = x[1];
        double vy = x[3];
        double vz = x[5];
        double v = std::sqrt(vx*vx + vy*vy + vz*vz);

        double scale_xy = std::exp(-v) * (q_sxy2_max - q_sxy2_min) + q_sxy2_min;
        double scale_z  = std::exp(-v) * (q_sz2_max  - q_sz2_min)  + q_sz2_min;

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        Eigen::Matrix2d Q_block;
        Q_block << 0.25 * dt4, 0.5 * dt3,
                   0.5  * dt3,       dt2;

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
        Q.block<2, 2>(0, 0) = Q_block * scale_xy;
        Q.block<2, 2>(2, 2) = Q_block * scale_xy;
        Q.block<2, 2>(4, 4) = Q_block * scale_z;

        return Q;
    };

    return { f, Q, jacobian_f };
}

/****************************************************************
 * @brief 三维非线性二次空气阻力模型（UKF用，无雅可比）
 ****************************************************************/
inline ProcessModel makeQuadraticDrag3D(double q_sxy_min, double q_sxy_max,
                                        double q_sz_min, double q_sz_max,
                                        double k, double /*m*/, double g = 9.8)
{
    double q_sxy2_min = std::pow(q_sxy_min, 2);
    double q_sxy2_max = std::pow(q_sxy_max, 2);
    double q_sz2_min = std::pow(q_sz_min, 2);
    double q_sz2_max = std::pow(q_sz_max, 2);

    // 微分方程: dx/dt = [vx, -k*vx*|v|, vy, -k*vy*|v|, vz, -k*vz*|v| - g]
    auto diff_x = [k, g](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
        double vx = x[1], vy = x[3], vz = x[5];
        double v = std::sqrt(vx*vx + vy*vy + vz*vz);
        Eigen::VectorXd dx(6);
        dx << vx,     -k * vx * v,
              vy,     -k * vy * v,
              vz,     -k * vz * v - g;
        return dx;
    };

    auto f = [diff_x](const Eigen::VectorXd& x, double dt) -> Eigen::VectorXd
    {
        auto k1 = diff_x(x);
        auto k2 = diff_x(x + 0.5 * k1 * dt);
        auto k3 = diff_x(x + 0.5 * k2 * dt);
        auto k4 = diff_x(x + k3 * dt);
        return x + (k1 + 2.0*k2 + 2.0*k3 + k4) * dt / 6.0;
    };

    auto Q = [q_sxy2_min, q_sxy2_max, q_sz2_min, q_sz2_max](const Eigen::VectorXd& x, double dt) -> Eigen::MatrixXd
    {
        double vx = x[1];
        double vy = x[3];
        double vz = x[5];
        double v = std::sqrt(vx*vx + vy*vy + vz*vz);

        double scale_xy = std::exp(-v) * (q_sxy2_max - q_sxy2_min) + q_sxy2_min;
        double scale_z  = std::exp(-v) * (q_sz2_max  - q_sz2_min)  + q_sz2_min;

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        Eigen::Matrix2d Q_block;
        Q_block << 0.25 * dt4, 0.5 * dt3,
                   0.5  * dt3,       dt2;

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
        Q.block<2, 2>(0, 0) = Q_block * scale_xy;
        Q.block<2, 2>(2, 2) = Q_block * scale_xy;
        Q.block<2, 2>(4, 4) = Q_block * scale_z;

        return Q;
    };

    return { f, Q, {} };
}

}  // namespace volleyball

#endif  // VOLLEYBALL_PROCESS_MODEL_HPP
