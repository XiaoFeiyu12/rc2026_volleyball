/*******************************************************************************
 * @file KF.hpp
 * @brief 卡尔曼滤波器基类与实现（EKF+UKF），用于排球状态估计
 *******************************************************************************/

#ifndef KF_HPP
#define KF_HPP

// STL
#include <vector>
// Eigen
#include <eigen3/Eigen/Dense>

#include "volleyball_track/measure_model.hpp"
#include "volleyball_track/process_model.hpp"

namespace volleyball
{

/****************************************************************
 * @class KalmanFilterBase 卡尔曼滤波器基类
 ****************************************************************/
class KalmanFilterBase
{
public:
  virtual ~KalmanFilterBase() = default;
  virtual void setState(const Eigen::VectorXd& x0) = 0;  // 设置初始状态
  virtual Eigen::VectorXd getState() = 0;                 // 获取当前状态
  virtual Eigen::VectorXd predict(double dt) = 0;         // 预测步骤
  virtual Eigen::VectorXd update(const Eigen::VectorXd& z) = 0;  // 更新步骤
};

/****************************************************************
 * @class EKF 扩展卡尔曼滤波器
 ****************************************************************/
class EKF : public KalmanFilterBase
{
public:
    EKF(const Eigen::MatrixXd& P0, ProcessModel pm, MeasureModel mm);
    void setState(const Eigen::VectorXd& x0) override;
    Eigen::VectorXd getState() override { return x_post_; }
    Eigen::VectorXd predict(double dt) override;
    Eigen::VectorXd update(const Eigen::VectorXd& z) override;

private:
    ProcessModel pm_;   // 过程模型
    MeasureModel mm_;   // 观测模型
    Eigen::VectorXd x_pred_;   // 先验状态
    Eigen::VectorXd x_post_;  // 后验状态
    Eigen::MatrixXd F_;       // 状态转移雅可比矩阵
    Eigen::MatrixXd H_;       // 观测雅可比矩阵
    Eigen::MatrixXd P_pred_;    // 先验误差协方差
    Eigen::MatrixXd P_post_;   // 后验误差协方差
    Eigen::MatrixXd Q_;       // 过程噪声协方差
    Eigen::MatrixXd R_;       // 观测噪声协方差
};

/****************************************************************
 * @class UKF 无迹卡尔曼滤波器
 ****************************************************************/
class UKF : public KalmanFilterBase
{
public:
    UKF(double alpha, double kappa, double beta, int n, int m, const Eigen::MatrixXd& P0,
        ProcessModel pm, MeasureModel mm);
    void setState(const Eigen::VectorXd& x0) override;
    Eigen::VectorXd getState() override { return x_post_; }
    Eigen::VectorXd predict(double dt) override;
    Eigen::VectorXd update(const Eigen::VectorXd& z) override;

private:
    std::vector<Eigen::VectorXd> generateSigmaPoints();  // 生成Sigma点
    ProcessModel pm_;   // 过程模型
    MeasureModel mm_;   // 观测模型

    int sigma_num_;   // Sigma点个数
    double alpha_;    // Sigma点传播范围
    double kappa_;    // 次要缩放参数
    double beta_;     // 高斯分布形状参数
    double lambda_;   // 缩放因子
    int n_;           // 状态维度
    int m_;           // 观测维度

    Eigen::VectorXd Wc_;  // 协方差权重
    Eigen::VectorXd Wm_;  // 均值权重

    Eigen::VectorXd x_pred_;   // 先验状态
    Eigen::VectorXd x_post_;  // 后验状态
    Eigen::MatrixXd P_pred_;   // 先验误差协方差
    Eigen::MatrixXd P_post_;  // 后验误差协方差
    Eigen::MatrixXd Q_;       // 过程噪声协方差
    Eigen::MatrixXd R_;       // 观测噪声协方差

    std::vector<Eigen::VectorXd> sigma_points_post_;  // 传播前Sigma点
    std::vector<Eigen::VectorXd> sigma_points_pred_;   // 传播后Sigma点
};

}  // namespace volleyball

#endif  // KF_HPP
