/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 *
 * This file serves as a ROS interface layer for the Eddie robot.
 */

#ifndef EDDIE_ROS_INTERFACE_HPP
#define EDDIE_ROS_INTERFACE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/ethercat.h"
#include "robif2b/functions/eddie_power_board.h"
#include "robif2b/functions/kelo_drive.h"
#include "robif2b/functions/kinova_gen3.h"

#include "eddie-ros/eddie_ros.fsm.hpp"

#define NUM_DRIVES 4
#define NUM_SLAVES 5
#define NUM_JOINTS 7

class PID {
  public:
	PID(double p_gain, double i_gain, double d_gain, double decay_rate = 0.0);

	double control(double error, double dt = 1.0);

  public:
	double err_integ;
	double err_last;
	double kp;
	double ki;
	double kd;
	double decay_rate;
};

class EddieRosInterface : public rclcpp::Node {
  public:
	explicit EddieRosInterface(const rclcpp::NodeOptions &options);

	~EddieRosInterface();

	struct EddieState {
		int num_drives;
		struct {
			struct timespec cycle_start;
			struct timespec cycle_end;
			long cycle_time_msr; // [us]
			long cycle_time_exp; // [us]
		} time;
		struct {
			struct robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
			struct robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
			struct robif2b_eddie_power_board_api_msr_pdo pb_msr_pdo;
			struct robif2b_eddie_power_board_api_cmd_pdo pb_cmd_pdo;
		} ecat_comm;
		struct {
			const char *ethernet_if;
			int error_code;
			int num_exposed_slaves;
			int num_found_slaves;
			int num_active_slaves;
			int slave_idx[NUM_SLAVES];
			const char *name[NUM_SLAVES];
			unsigned int prod_code[NUM_SLAVES];
			size_t input_size[NUM_SLAVES];
			size_t output_size[NUM_SLAVES];
			bool is_connected[NUM_SLAVES];
		} ecat;
		struct {
			double pvt_off[NUM_DRIVES];
			double pvt_pos[NUM_DRIVES];
			double pvt_vel[NUM_DRIVES];
			double whl_pos[NUM_DRIVES * 2];
			double whl_vel[NUM_DRIVES * 2];
			double imu_ang_vel[NUM_DRIVES * 3];
			double imu_lin_acc[NUM_DRIVES * 3];
			double bat_volt;
			double bat_cur;
			double bat_pwr;
			uint64_t time_stamp;
			uint16_t status;
		} kelo_msr;
		struct {
			enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES];
			double vel[NUM_DRIVES * 2];
			double trq[NUM_DRIVES * 2];
			double cur[NUM_DRIVES * 2];
			double max_current[NUM_DRIVES * 2];
			double trq_const[NUM_DRIVES * 2];
		} kelo_cmd;
		struct {
			bool success;
			enum robif2b_ctrl_mode ctrl_mode;
			double pos_msr[NUM_JOINTS];
			double vel_msr[NUM_JOINTS];
			double eff_msr[NUM_JOINTS];
			double cur_msr[NUM_JOINTS];
			double pos_cmd[NUM_JOINTS];
			double vel_cmd[NUM_JOINTS];
			double eff_cmd[NUM_JOINTS];
			double cur_cmd[NUM_JOINTS];
			double imu_ang_vel_msr[3];
			double imu_lin_acc_msr[3];
		} kinova_rightarm_state;
		struct {
			bool success;
			enum robif2b_ctrl_mode ctrl_mode;
			double pos_msr[NUM_JOINTS];
			double vel_msr[NUM_JOINTS];
			double eff_msr[NUM_JOINTS];
			double cur_msr[NUM_JOINTS];
			double pos_cmd[NUM_JOINTS];
			double vel_cmd[NUM_JOINTS];
			double eff_cmd[NUM_JOINTS];
			double cur_cmd[NUM_JOINTS];
			double imu_ang_vel_msr[3];
			double imu_lin_acc_msr[3];
		} kinova_leftarm_state;
	};
	EddieState eddie_state;

	struct robif2b_ethercat ecat;
	struct robif2b_kelo_drive_encoder drive_enc;
	struct robif2b_kelo_drive_imu imu;
	struct robif2b_kelo_drive_actuator wheel_act;
	struct robif2b_eddie_power_board power_board;
	struct robif2b_kinova_gen3_nbx kinova_rightarm;
	struct robif2b_kinova_gen3_nbx kinova_leftarm;

	void *input[NUM_SLAVES];
	const void *output[NUM_SLAVES];

	// dynamic parameters
	std::string param_ethernet_if;

	// dynamic parametes methods
	rcl_interfaces::msg::SetParametersResult
	parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

	void declare_all_parameters();

	void get_all_parameters();

	// sm methods
	void configure(events *eventData, EddieState *eddie_state);
	void idle(events *eventData, EddieState *eddie_state);
	void compile(events *eventData, EddieState *eddie_state);
	void execute(events *eventData, EddieState *eddie_state);

	void compute_gravity_comp(events *eventData, EddieState *eddie_state);

	void fsm_behavior(events *eventData, EddieState *eddie_state);

  public:
	void run_fsm();

  private:
	KDL::Tree tree;
	KDL::Chain leftarm_chain;
	KDL::Chain rightarm_chain;

	int num_jnts_leftarm;
	int num_segs_leftarm;
	KDL::Twist root_acc_leftarm;
	KDL::JntArray q_leftarm;
	KDL::JntArray qd_leftarm;
	KDL::JntArray qdd_leftarm;
	KDL::JntArray tau_ctrl_leftarm;
	KDL::Wrenches f_ext_leftarm;
	std::unique_ptr<KDL::ChainFkSolverPos_recursive> fpk_solver_leftarm;
	std::unique_ptr<KDL::ChainFkSolverVel_recursive> fvk_solver_leftarm;
	std::unique_ptr<KDL::ChainIdSolver_RNE> rne_id_solver_leftarm;

	int num_jnts_rightarm;
	int num_segs_rightarm;
	KDL::Twist root_acc_rightarm;
	KDL::JntArray q_rightarm;
	KDL::JntArray qd_rightarm;
	KDL::JntArray qdd_rightarm;
	KDL::JntArray tau_ctrl_rightarm;
	KDL::Wrenches f_ext_rightarm;
	std::unique_ptr<KDL::ChainFkSolverPos_recursive> fpk_solver_rightarm;
	std::unique_ptr<KDL::ChainFkSolverVel_recursive> fvk_solver_rightarm;
	std::unique_ptr<KDL::ChainIdSolver_RNE> rne_id_solver_rightarm;
};

#endif // EDDIE_ROS_INTERFACE_HPP