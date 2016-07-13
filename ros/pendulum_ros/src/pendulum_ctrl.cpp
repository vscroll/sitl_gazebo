#include "pendulum_ctrl.h"
#include "pendulum_dynamic.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

PendulumCtrl::PendulumCtrl()
	:_nh( "~" ),
	_started(false),
	_rate(200),
	_pendulum_l(DEFAULT_PENDULUM_L),
	_reset_pose(false) {
	
	_pendulum_x_pid.setGains(DEFAULT_PENDULUM_X_P, DEFAULT_PENDULUM_X_I, DEFAULT_PENDULUM_X_D);
	_pendulum_x_pid.setGains(DEFAULT_PENDULUM_Y_P, DEFAULT_PENDULUM_Y_I, DEFAULT_PENDULUM_Y_D);

	dynamic_reconfigure::Server<pendulum_ros::PendulumConfig>::CallbackType f;
	f = boost::bind(&PendulumCtrl::cfg_pid_callback, this, _1, _2);
	_cfg_server.setCallback(f);

	reset_pose();
}

PendulumCtrl::~PendulumCtrl() {

}

void PendulumCtrl::cfg_pid_callback(pendulum_ros::PendulumConfig &config, uint32_t level) {

	ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %d %d", 
			config.pendulum_l,
			config.pendulum_x_p,
			config.pendulum_x_i, 
			config.pendulum_x_d,
			config.pendulum_y_p,
			config.pendulum_y_i, 
			config.pendulum_y_d,
			config.pendulum_cmd,
			config.vehicle_cmd
			);

	_pendulum_l = config.pendulum_l;
	_pendulum_x_pid.setGains(config.pendulum_x_p, config.pendulum_x_i, config.pendulum_x_d);
	_pendulum_y_pid.setGains(config.pendulum_y_p, config.pendulum_y_i, config.pendulum_y_d);

	if (config.pendulum_cmd == PENDULUM_CMD_START) {
		_reset_pose = true;
		reset_pose();

		start();
	} else if (config.pendulum_cmd == PENDULUM_CMD_STOP) {
		stop();
	} else {
	}

	if (config.vehicle_cmd == VEHICLE_CMD_ARMED) {
		armed();
	} else if (config.vehicle_cmd == VEHICLE_CMD_DISARMED) {
		disarmed();
	} else if (config.vehicle_cmd == VEHICLE_CMD_TAKEOFF) {
		takeoff();
	} else if (config.vehicle_cmd == VEHICLE_CMD_POSCTL) {
		set_posctl();
	} else if (config.vehicle_cmd == VEHICLE_CMD_OFFBOARD) {
		set_offboard();
	} else {
	}
}

void PendulumCtrl::reset_pose() {
	memset(&_pose.header.stamp, 0, sizeof(_pose.header.stamp));
}

bool PendulumCtrl::open() {
	ROS_INFO("open");
	pthread_create(&_thread, NULL, &PendulumCtrl::threadRun, this);

	_pendulum_sub = _nh.subscribe("/FMA/Pendulum/Pose", 100, &PendulumCtrl::pendulum_pose_callback, this);

    _throttle_pub = _nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10);
//	_attitude_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
	_attitude_pub = _nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 10);

	_arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	_set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void PendulumCtrl::close() {
	ROS_INFO("close");
	if (_started) {
		stop();
	}
}

void* PendulumCtrl::threadRun(void* argument) {
	((PendulumCtrl*)argument)->threadFun();
	return NULL;
}

void PendulumCtrl::threadFun() {
	ROS_INFO("threadFun");
	while(ros::ok()) {
		if (!_started) {
			continue;
		}
/*
		_pose.header = _pose_local.header;
		_pose.position = _pose_local.position;
		_pose.velocity = _pose_local.velocity;
		_pose.vel_acc = _pose_local.vel_acc;
*/
		_pose = _pose_local;

		if (_reset_pose) {
			ROS_INFO("reset pose");
			_reset_pose = false;
			_pendulum_x_pid.reset();
			_pendulum_y_pid.reset();

			_pendulum_x_pid.setReference(_pose.position.x);
			_pendulum_y_pid.setReference(_pose.position.y);
		} else {
			_pendulum_x_pid.setFeedback(_pose.position.x);
			_pendulum_y_pid.setFeedback(_pose.position.y);

			_pendulum_output_x = _pendulum_x_pid.getOutput();
			_pendulum_output_y = _pendulum_y_pid.getOutput();

			double vehicle_vel_acc_x = 0.0;
			double vehicle_vel_acc_y = 0.0;
			PendulumDynamic::formula_12(_pendulum_l,
								_pendulum_output_x,
								_pendulum_output_y,
								_pose.velocity,
								_pose.vel_acc,
								&vehicle_vel_acc_x,
								&vehicle_vel_acc_y);
			ROS_INFO("formula 12:%f %f", vehicle_vel_acc_x, vehicle_vel_acc_y);

			double angle_x = 0.0;
			double angle_y = 0.0;
			double a = 0;
			PendulumDynamic::formula_5_2(vehicle_vel_acc_x, vehicle_vel_acc_y,
								&angle_x, &angle_y, &a);
			ROS_INFO("formula 5:%f %f %f %f", angle_x, angle_y, a, a/(1.5*PendulumDynamic::g));

			double vehicle_rate_x = 0.0;
			double vehicle_rate_y = 0.0;
			PendulumDynamic::formula_7(angle_x, angle_y,
									angle_x, angle_y,
									&vehicle_rate_x, &vehicle_rate_y);
			ROS_INFO("formula 7:%f %f", vehicle_rate_x, vehicle_rate_y);

			if (_throttle_pub) {
				std_msgs::Float64 throttle;
				throttle.data = a/(1.5*PendulumDynamic::g); // 0~1
				_throttle_pub.publish(throttle);
			}

			if (_attitude_pub) {
				geometry_msgs::TwistStamped attitude;
				attitude.twist.angular.x = vehicle_rate_x;
				attitude.twist.angular.y = vehicle_rate_y;
				attitude.twist.angular.z = 0;
				_attitude_pub.publish(attitude);
			}
		}

		_rate.sleep();
	}

	ROS_INFO("exit threadFun");
	_started = false;
}

void PendulumCtrl::start() {
	ROS_INFO("start");
	_started = true;
}

void PendulumCtrl::stop() {
	ROS_INFO("stop");
	_started = false;
}

bool PendulumCtrl::running() {
	return _started;
}

void PendulumCtrl::armed() {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	if (_arming_client) {
		if(_arming_client.call(arm_cmd) &&
				arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
		}
	}
}

void PendulumCtrl::disarmed() {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = false;
	if (_arming_client) {
		if (_arming_client.call(arm_cmd) &&
				arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
		}
	}
}

void PendulumCtrl::takeoff() {
}

void PendulumCtrl::set_posctl() {
}

void PendulumCtrl::set_offboard() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	if (_set_mode_client) {
		if (_set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success){
			ROS_INFO("Offboard enabled");
		}
	}
}

void PendulumCtrl::pendulum_pose_callback(const fmaros_msgs::PendulumPose::ConstPtr& msg) {
/*
	_pose_local.header = msg->header;
	_pose_local.position = msg->position;
	_pose_local.velocity = msg->velocity;
	_pose_local.vel_acc = msg->vel_acc;
*/
	_pose_local = *msg;

/*
	ROS_INFO("Pendulum Pose: %f %f %f %f %f %f %f %f %f",
		_pose_local.position.x, _pose_local.position.y, _pose_local.position.z,
		_pose_local.velocity.x, _pose_local.velocity.y, _pose_local.velocity.z,
		_pose_local.vel_acc.x, _pose_local.vel_acc.y, _pose_local.vel_acc.z);
*/
}

void PendulumCtrl::vehicle_pose_callback(const fmaros_msgs::VehiclePose::ConstPtr& msg) {

}

