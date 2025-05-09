/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 */

#include "eddie-ros/interface.hpp"

void EddieRosInterface::declare_all_parameters() {
	rcl_interfaces::msg::ParameterDescriptor ethercat_if_desc_;
	ethercat_if_desc_.description = "EtherCAT interface name";
	ethercat_if_desc_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
	this->declare_parameter("ethernet_if", "eno1", ethercat_if_desc_);
	this->get_parameter("ethernet_if", param_ethernet_if);
}

void EddieRosInterface::get_all_parameters() {
	this->get_parameter("ethernet_if", param_ethernet_if);
}

// rcl_interfaces::msg::SetParametersResult
//   EddieRosInterface::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
// {
// 	rcl_interfaces::msg::SetParametersResult result;
// 	result.successful = true;
// 	result.reason = "Success";

// 	for (const auto &param : parameters) {
// 		if (param.get_name() == "ethernet_if") {
// 			this->param_ethernet_if = param.get_value<std::string>();
// 		}
// 	}

// 	return result;
// }