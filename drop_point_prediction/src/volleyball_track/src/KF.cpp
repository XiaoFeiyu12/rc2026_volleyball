/*******************************************************************************
 * @file KF.cpp
 * @brief 扩展卡尔曼滤波（EKF）与无迹卡尔曼滤波（UKF）实现
 *******************************************************************************/

#include "volleyball_track/KF.hpp"

#include "rclcpp/rclcpp.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief EKF构造函数：初始化状态与协方差
 *****************************************************************/
EKF::EKF(const Eigen::MatrixXd &P0, ProcessModel pm, MeasureModel mm)
	: pm_(std::move(pm)),
	  mm_(std::move(mm)),
	  x_pred_(Eigen::VectorXd::Zero(6)),
	  x_post_(Eigen::VectorXd::Zero(6)),
	  P_post_(P0)
{
	P_pred_ = P_post_;
}

/*****************************************************************
 * @brief 设置EKF初始状态
 *****************************************************************/
void EKF::set_state(const Eigen::VectorXd &x0)
{
	x_post_ = x0;
	x_pred_ = x_post_;
}

/*****************************************************************
 * @brief EKF预测步骤：线性化状态转移，预测先验状态与协方差
 *****************************************************************/
Eigen::VectorXd EKF::predict(double dt)
{
	// 更新雅可比矩阵与噪声矩阵
	F_ = pm_.jacobian_f(x_post_, dt);
	Q_ = pm_.Q(x_post_, dt);
	// 预测状态
	x_pred_ = pm_.f(x_post_, dt);
	// 预测协方差：P = F * P * F^T + Q
	P_pred_ = F_ * P_post_ * F_.transpose() + Q_;

	// 将后验设为先验
	x_post_ = x_pred_;
	P_post_ = P_pred_;

	return x_pred_;
}

/*****************************************************************
 * @brief EKF更新步骤：利用观测修正先验状态
 *****************************************************************/
Eigen::VectorXd EKF::update(const Eigen::VectorXd &z)
{
	// 更新观测雅可比与噪声矩阵
	H_ = mm_.jacobian_h(x_pred_);
	R_ = mm_.R(z);
	// 卡尔曼增益：K = P * H^T * (H*P*H^T + R)^(-1)
	Eigen::MatrixXd S = (H_ * P_pred_ * H_.transpose() + R_);

	// 进行马式距离检测是否异常
	Eigen::VectorXd innovation = z - mm_.h(x_pred_);  // 3D = 3D - h(6D)
	Eigen::MatrixXd S_inv = S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
	double maha_distance = innovation.transpose() * S_inv * innovation;
	RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "马式距离:%f", maha_distance);
	// if (maha_distance >  7.815) // dof为3的卡方统计量X^2(3)
	// {
	//     return x_post_;
	// }

	Eigen::MatrixXd K = P_pred_ * H_.transpose() * S_inv;
	// 状态修正
	x_post_ = x_pred_ + K * (innovation);
	// 协方差更新（Joseph格式，提升数值稳定性）
	const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_post_.size(), x_post_.size());
	Eigen::MatrixXd A = I - K * H_;
	P_post_ = A * P_pred_ * A.transpose() + K * R_ * K.transpose();

	return x_post_;
}

/*****************************************************************
 * @brief UKF构造函数：初始化参数与Sigma点权重
 *****************************************************************/
UKF::UKF(double alpha, double kappa, double beta, int n, int m, const Eigen::MatrixXd &P0, ProcessModel pm,
		 MeasureModel mm)
	: pm_(std::move(pm)), mm_(std::move(mm)), alpha_(alpha), kappa_(kappa), beta_(beta), n_(n), m_(m), P_post_(P0)
{
	P_pred_ = P_post_;
	lambda_ = alpha * alpha * (n_ + kappa_) - n_;

	sigma_num_ = 2 * n_ + 1;
	Wc_.resize(sigma_num_);
	Wm_.resize(sigma_num_);
	Wm_[0] = lambda_ / (n_ + lambda_);
	Wc_[0] = Wm_[0] + (1 - alpha_ * alpha_ + beta_);
	for (int i = 1; i < sigma_num_; i++)
	{
		Wc_[i] = Wm_[i] = 0.5 / (n_ + lambda_);
	}

	sigma_points_post_ = std::vector<Eigen::VectorXd>(sigma_num_);
	sigma_points_pred_ = std::vector<Eigen::VectorXd>(sigma_num_);
}

/*****************************************************************
 * @brief 设置UKF初始状态
 *****************************************************************/
void UKF::set_state(const Eigen::VectorXd &x0)
{
	x_post_ = x0;
	x_pred_ = x_post_;
}

/*****************************************************************
 * @brief 基于当前后验协方差生成Sigma点
 *****************************************************************/
std::vector<Eigen::VectorXd> UKF::generate_sigma_points()
{
	std::vector<Eigen::VectorXd> sigma_points(sigma_num_);
	/*计算协方差矩阵*/
	Eigen::LLT<Eigen::MatrixXd> llt(P_post_);
	/*这一步主要是解决有时候由于计算误差导致协方差矩阵并不是正定矩阵*/
	if (llt.info() != Eigen::Success)
	{
		Eigen::MatrixXd P_fix = P_post_ + 1e-9 * Eigen::MatrixXd::Identity(n_, n_);
		llt.compute(P_fix);
	}
	/*求出下三角矩阵作为平方根*/
	Eigen::MatrixXd S = llt.matrixL();
	/*计算sigma点*/
	sigma_points[0] = x_post_;
	for (int i = 0; i < n_; i++)
	{
		Eigen::VectorXd col_process = std::sqrt(n_ + lambda_) * S.col(i);
		sigma_points[1 + i] = x_post_ + col_process;
		sigma_points[1 + i + n_] = x_post_ - col_process;
	}

	return sigma_points;
}

/*****************************************************************
 * @brief UKF预测步骤：传播Sigma点，加权计算先验状态与协方差
 *****************************************************************/
Eigen::VectorXd UKF::predict(double dt)
{
	/*生成sigma点*/
	sigma_points_post_ = generate_sigma_points();
	/*传播sigma点*/
	for (int i = 0; i < sigma_num_; i++)
	{
		sigma_points_pred_[i] = pm_.f(sigma_points_post_[i], dt);
	}
	/*取加权平均值作为预测值*/
	x_pred_.setZero();
	P_pred_.setZero();
	for (int i = 0; i < sigma_num_; i++)
	{
		x_pred_ += Wm_[i] * sigma_points_pred_[i];
	}
	for (int i = 0; i < sigma_num_; i++)
	{
		Eigen::VectorXd error = sigma_points_pred_[i] - x_pred_;
		P_pred_ += Wc_[i] * error * error.transpose();
	}
	P_pred_ += pm_.Q(x_post_, dt);

	x_post_ = x_pred_;
	P_post_ = P_pred_;

	return x_pred_;
}

/*****************************************************************
 * @brief UKF更新步骤：利用观测修正先验状态
 *****************************************************************/
Eigen::VectorXd UKF::update(const Eigen::VectorXd &z)
{
	/*传播观测值的sigma点,并计算观测预测均值*/
	std::vector<Eigen::VectorXd> zeta(sigma_num_);
	Eigen::VectorXd z_pri_mean_(m_);
	z_pri_mean_.setZero();
	for (int i = 0; i < sigma_num_; i++)
	{
		zeta[i] = mm_.h(sigma_points_pred_[i]);
		z_pri_mean_ += Wm_[i] * zeta[i];
	}
	/*计算新息协方差和交叉协方差*/
	Eigen::MatrixXd S(m_, m_);
	Eigen::MatrixXd Pxz(n_, m_);
	S.setZero();
	Pxz.setZero();
	for (int i = 0; i < sigma_num_; i++)
	{
		Eigen::VectorXd error = zeta[i] - z_pri_mean_;
		S += Wc_[i] * error * error.transpose();
		Pxz += Wc_[i] * (sigma_points_pred_[i] - x_pred_) * error.transpose();
	}
	S += mm_.R(z);
	/*计算卡尔曼增益*/
	Eigen::MatrixXd K = Pxz * S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
	/*更新*/
	x_post_ = x_pred_ + K * (z - z_pri_mean_);
	P_post_ = P_pred_ - K * S * K.transpose();

	return x_post_;
}

}  // namespace volleyball
