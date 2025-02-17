//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
// This file was automatically generated by the Team 302 code generator version 20.25.00.02
// Generated on Tuesday, January 28, 2025 10:59:07 PM

#pragma once

class RobotElementNames
{

public:
	enum LED_USAGE
	{
		UNKNOWN_LED = -1,
		LED,
		MAX_LED
	};

	enum LED_SEGMENT_USAGE
	{
		UNKNOWN_LED_SEGMENT = -1,
		MAX_LED_SEGMENT
	};

	enum CAMERA_USAGE
	{
		UNKNOWN_CAMERA = -1,
		MAX_CAMERA
	};

	enum MOTOR_CONTROLLER_USAGE
	{
		UNKNOWN_MOTOR_CONTROLLER = -1,
		CLIMBER_MANAGER_CLIMBER,
		DRAGON_TALE_ARM,
		DRAGON_TALE_ELEVATOR_LEADER,
		DRAGON_TALE_CORAL,
		DRAGON_TALE_ALGAE,
		DRAGON_TALE_ELEVATOR_FOLLOWER,
		INTAKE_MANAGER_INTAKE,
		INTAKE_MANAGER_EXTENDER,
		MAX_MOTOR_CONTROLLER
	};

	enum FEEDBACK_SENSOR_CONFIG_BASE_USAGE
	{
		UNKNOWN_FEEDBACK_SENSOR_CONFIG_BASE = -1,
		MAX_FEEDBACK_SENSOR_CONFIG_BASE
	};

	enum CANCODER_INSTANCE_USAGE
	{
		UNKNOWN_CANCODER_INSTANCE = -1,
		MAX_CANCODER_INSTANCE
	};

	enum PID_USAGE
	{
		UNKNOWN_PID = -1,
		MAX_PID
	};

	enum PDP_USAGE
	{
		UNKNOWN_PDP = -1,
		PDP,
		MAX_PDP
	};

	enum PIGEON_USAGE
	{
		UNKNOWN_PIGEON = -1,
		PIGEON_ROBOT_CENTER,
		MAX_PIGEON
	};

	enum PCM_USAGE
	{
		UNKNOWN_PCM = -1,
		MAX_PCM
	};

	enum ANALOG_INPUT_USAGE
	{
		UNKNOWN_ANALOG_INPUT = -1,
		MAX_ANALOG_INPUT
	};

	enum CHASSIS_USAGE
	{
		UNKNOWN_CHASSIS = -1,
		MAX_CHASSIS
	};

	enum DIGITAL_INPUT_USAGE
	{
		UNKNOWN_DIGITAL_INPUT = -1,
		DRAGON_TALE_CORAL_IN_SENSOR,
		DRAGON_TALE_CORAL_OUT_SENSOR,
		DRAGON_TALE_ALGAE_SENSOR,
		INTAKE_MANAGER_INTAKE_SENSOR,
		MAX_DIGITAL_INPUT
	};

	enum SWERVE_MODULE_USAGE
	{
		UNKNOWN_SWERVE_MODULE = -1,
		MAX_SWERVE_MODULE
	};

	enum CANCODER_USAGE
	{
		UNKNOWN_CANCODER = -1,
		DRAGON_TALE_ARM_ANGLE_SENSOR,
		DRAGON_TALE_ELEVATOR_HEIGHT_SENSOR,
		MAX_CANCODER
	};

	enum SOLENOID_USAGE
	{
		UNKNOWN_SOLENOID = -1,
		MAX_SOLENOID
	};

	enum SERVO_USAGE
	{
		UNKNOWN_SERVO = -1,
		MAX_SERVO
	};

	enum COLOR_SENSOR_USAGE
	{
		UNKNOWN_COLOR_SENSOR = -1,
		MAX_COLOR_SENSOR
	};

	enum ROBORIO_USAGE
	{
		UNKNOWN_ROBORIO = -1,
		MAX_ROBORIO
	};

	enum TALONTACH_USAGE
	{
		UNKNOWN_TALONTACH = -1,
		MAX_TALONTACH
	};

	enum MOTOR_CONTROL_DATA_USAGE
	{
		UNKNOWN_MOTOR_CONTROL_DATA = -1,
		CLIMBER_MANAGER_POSITION_DEGREE,
		DRAGON_TALE_POSITION_INCH,
		DRAGON_TALE_POSITION_DEGREE,
		DRAGON_TALE_PERCENT_OUTPUT,
		INTAKE_MANAGER_PERCENT_OUTPUT,
		INTAKE_MANAGER_POSITION_DEGREE,
		MAX_MOTOR_CONTROL_DATA
	};

	enum MOTOR_CONTROL_DATA_LINK_USAGE
	{
		UNKNOWN_MOTOR_CONTROL_DATA_LINK = -1,
		MAX_MOTOR_CONTROL_DATA_LINK
	};

	enum MOTOR_TARGET_USAGE
	{
		UNKNOWN_MOTOR_TARGET = -1,
		MAX_MOTOR_TARGET
	};

private:
	RobotElementNames() = delete;
	~RobotElementNames() = delete;
};
