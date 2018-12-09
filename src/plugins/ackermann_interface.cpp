/****************************************************************************
 *
 *   Copyright (c) 2018 Mohammed Kabir. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Mohammed Kabir nor the names of their contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <mavros/mavros_plugin.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

namespace mavros {
namespace rover_plugins{

class AckermannInterfacePlugin : public plugin::PluginBase {
public:
	AckermannInterfacePlugin() : PluginBase(),
		ai_nh("~ackermann_interface")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		manual_sp_sub_ = ai_nh.subscribe("/ecu_interface/control/ackermann_cmd_manual", 10, &AckermannInterfacePlugin::manual_sp_cb, this);
		auto_sp_sub_ = ai_nh.subscribe("/ecu_interface/control/ackermann_cmd_auto", 10, &AckermannInterfacePlugin::auto_sp_cb, this);

	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle ai_nh;

	ros::Subscriber manual_sp_sub_;
	ros::Subscriber auto_sp_sub_;

	double last_manual_timestamp_;
	bool autonomous_control_;

	void manual_sp_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
	{
		
		last_manual_timestamp_ = msg->header.stamp.toSec();

		mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED setpoint{};

		setpoint.time_boot_ms = msg->header.stamp.toNSec() / 1000000;

		setpoint.target_system = m_uas->get_tgt_system();
		setpoint.target_component = m_uas->get_tgt_component();

		setpoint.vx = msg->drive.speed;
		setpoint.afx = msg->drive.acceleration;
		setpoint.yaw = msg->drive.steering_angle;
		setpoint.yaw_rate = msg->drive.steering_angle_velocity;

		UAS_FCU(m_uas)->send_message_ignore_drop(setpoint);

	}

	void auto_sp_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
	{
		
		if(ros::Time::now().toSec() - last_manual_timestamp_ < 0.001) {
			if(autonomous_control_) {
				ROS_INFO("Switching to manual control");
				autonomous_control_ = false;
			}
			return;
		} else {
			if(!autonomous_control_) {
				ROS_INFO("Switching to autonomous control");
				autonomous_control_ = true;
			}
		}

		mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED setpoint{};

		setpoint.time_boot_ms = msg->header.stamp.toNSec() / 1000000;

		setpoint.target_system = m_uas->get_tgt_system();
		setpoint.target_component = m_uas->get_tgt_component();

		// Magic key to enable autonomous control
		setpoint.type_mask = 45915;

		setpoint.vx = msg->drive.speed;
		setpoint.afx = msg->drive.acceleration;
		setpoint.yaw = msg->drive.steering_angle;
		setpoint.yaw_rate = msg->drive.steering_angle_velocity;

		UAS_FCU(m_uas)->send_message_ignore_drop(setpoint);

	}



};
}	// namespace rover_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::rover_plugins::AckermannInterfacePlugin, mavros::plugin::PluginBase)
