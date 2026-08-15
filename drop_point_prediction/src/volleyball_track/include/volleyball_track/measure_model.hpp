/*******************************************************************************
 * @file measure_model.hpp
 * @brief 观测模型定义，用于卡尔曼滤波器的量测更新
 *******************************************************************************/

#ifndef VOLLEYBALL_MEASURE_MODEL_HPP
#define VOLLEYBALL_MEASURE_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>

namespace volleyball
{

struct MeasureModel
{
	std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h;
	std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> R;
	std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> jacobian_h;
};

struct MeasureModelContext
{
	Eigen::Vector3d pos_in_camera = Eigen::Vector3d::Zero();
	Eigen::Matrix3d camera2odom_rot = Eigen::Matrix3d::Identity();
};

// ===================================================================
//  工厂函数
// ===================================================================
/****************************************************************
 * @brief 三维位置模型
 ****************************************************************/
inline MeasureModel make_position_3d(int sigma_pixel, double sigma_depth_gain, double sigma_depth_const, double fx,
								   double fy, std::shared_ptr<MeasureModelContext> ctx)
{
	auto h = [](const Eigen::VectorXd &x) -> Eigen::VectorXd
	{
		Eigen::VectorXd z(3);
		z << x[0], x[2], x[4];
		return z;
	};

	double sigma_u = sigma_pixel;
	double sigma_v = sigma_pixel;

	auto R = [=](const Eigen::VectorXd &) -> Eigen::MatrixXd
	{
		Eigen::MatrixXd R_mat, R_xyz = Eigen::MatrixXd::Zero(3, 3);
		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 3);
		double cam_x = ctx->pos_in_camera[0];
		double cam_y = ctx->pos_in_camera[1];
		double cam_z = ctx->pos_in_camera[2];
		// clang-format off
        J << cam_z / fx, 0.0 , cam_x / cam_z,
             0.0, cam_z / fy , cam_y / cam_z,
             0.0 , 0.0 , 1.0;
		// clang-format on
		double sigma_depth = sigma_depth_gain * cam_z * cam_z + sigma_depth_const;

		Eigen::Vector3d diag_vec(3);
		diag_vec << sigma_u * sigma_u, sigma_v * sigma_v, sigma_depth * sigma_depth;
		Eigen::Matrix3d R_rpy = diag_vec.asDiagonal();
		R_xyz = J * R_rpy * J.transpose();
		auto camera2odom_rot = ctx->camera2odom_rot;
		R_mat = camera2odom_rot * R_xyz * camera2odom_rot.transpose();
		return R_mat;
	};

	auto jacobian_h = [](const Eigen::VectorXd &) -> Eigen::MatrixXd
	{
		Eigen::MatrixXd H(3, 6);
		// clang-format off
        H << 1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 1, 0;
		// clang-format on
		return H;
	};

	return {h, R, jacobian_h};
}

}  // namespace volleyball

#endif	// VOLLEYBALL_MEASURE_MODEL_HPP
