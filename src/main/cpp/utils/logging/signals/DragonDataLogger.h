
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

#pragma once
#include <string>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/time.h"
#include "utils/logging/signals/DragonDataLoggerSignals.h"
#include "magic_enum/magic_enum.hpp"

using namespace std;

class DragonDataLogger
{
public:
    DragonDataLogger();
    virtual ~DragonDataLogger() = default;

    virtual void DataLog(uint64_t timestamp) = 0;

    // when adding a signal:
    // - Add to the appropriate enum
    // - Add a signal variable below
    // - Construct the signal in DragonDataLoggerSignals::DragonDataLoggerSignals()
    // - Add Case statement in the appropriate helper method in DragonDataLogger

    enum BoolSignals
    {
        IS_BROWNOUT
    };

    enum DoubleSignals
    {
        CHASSIS_STORED_HEADING_DEGREES,
        CHASSIS_YAW_DEGREES,
        ELECTRICAL_VOLTAGE,
        ELECTRICAL_CURRENT,
        ELECTRICAL_POWER,
        LIMELIGHT_TV_1,
        LIMELIGHT_TX_1,
        LIMELIGHT_TY_1,
        LIMELIGHT_FIDUCIAL_ID_1,
        BATTERY_VOLTAGE,
        BROWNOUT_VOLTAGE,
        INPUT_VOLTAGE,
        INPUT_CURRENT,
        CPU_TEMP,
        LEFT_FRONT_SWERVE_STEER_POWER,
        LEFT_FRONT_SWERVE_STEER_CURRENT,
        LEFT_FRONT_SWERVE_STEER_TOTALPOWER,
        LEFT_FRONT_SWERVE_STEER_WATT_HOURS,
        LEFT_FRONT_SWERVE_DRIVE_POWER,
        LEFT_FRONT_SWERVE_DRIVE_CURRENT,
        LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_FRONT_SWERVE_STEER_POWER,
        RIGHT_FRONT_SWERVE_STEER_CURRENT,
        RIGHT_FRONT_SWERVE_STEER_TOTALPOWER,
        RIGHT_FRONT_SWERVE_STEER_WATT_HOURS,
        RIGHT_FRONT_SWERVE_DRIVE_POWER,
        RIGHT_FRONT_SWERVE_DRIVE_CURRENT,
        RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        LEFT_BACK_SWERVE_STEER_POWER,
        LEFT_BACK_SWERVE_STEER_CURRENT,
        LEFT_BACK_SWERVE_STEER_TOTALPOWER,
        LEFT_BACK_SWERVE_STEER_WATT_HOURS,
        LEFT_BACK_SWERVE_DRIVE_POWER,
        LEFT_BACK_SWERVE_DRIVE_CURRENT,
        LEFT_BACK_SWERVE_DRIVE_TOTALPOWER,
        LEFT_BACK_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_BACK_SWERVE_STEER_POWER,
        RIGHT_BACK_SWERVE_STEER_CURRENT,
        RIGHT_BACK_SWERVE_STEER_TOTALPOWER,
        RIGHT_BACK_SWERVE_STEER_WATT_HOURS,
        RIGHT_BACK_SWERVE_DRIVE_POWER,
        RIGHT_BACK_SWERVE_DRIVE_CURRENT,
        RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS,
        SWERVE_CHASSIS_TOTAL_CURRENT,
        SWERVE_CHASSIS_WATT_HOURS
    };

    enum StringSignals
    {
        CHASSIS_HEADING_STATE,
        CHASSIS_DRIVE_STATE,
        AUTON_PATH_NAME
    };

    enum PoseSingals
    {
        CURRENT_CHASSIS_POSE2D,
        CURRENT_CHASSIS_LIMELIGHT_POSE3D,
        CURRENT_CHASSIS_LIMELIGHT2_POSE3D,
        CURRENT_CHASSIS_QUEST_POSE2D,
        VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE,
        ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE

    };

    enum ChassisSpeedSignals
    {
        TARGET_SPEEDS,
        ACTUAL_SPEEDS
    };

    enum SwerveStateSingals
    {
        TARGET_LEFT_FRONT_STATE,
        TARGET_RIGHT_FRONT_STATE,
        TARGET_LEFT_BACK_STATE,
        TARGET_RIGHT_BACK_STATE,
        ACTUAL_LEFT_FRONT_STATE,
        ACTUAL_RIGHT_FRONT_STATE,
        ACTUAL_LEFT_BACK_STATE,
        ACTUAL_RIGHT_BACK_STATE
    };

    // initialize these signals in the constructor

    string m_pose2dUnits = "X, Y, Rotation";

    string m_frontLeftTargetSpeedPath = "/Chassis/FrontLeftModule/TargetState/Speed";
    string m_frontLeftTargetAnglePath;
    string m_frontRightTargetSpeedPath = "/Chassis/FrontRightModule/TargetState/Speed";
    string m_frontRightTargetAnglePath;
    string m_backLeftTargetSpeedPath = "/Chassis/BackLeftModule/TargetState/Speed";
    string m_backLeftTargetAnglePath = "/Chassis/BackLeftModule/TargetState/Angle";
    string m_backRightTargetSpeedPath = "/Chassis/BackRightModule/TargetState/Speed";
    string m_backRightTargetAnglePath = "/Chassis/BackRightModule/TargetState/Angle";
    string m_frontLeftActualSpeedPath = "/Chassis/FrontLeftModule/ActualState/Speed";
    string m_frontLeftActualAnglePath = "/Chassis/FrontLeftModule/ActualState/Angle";
    string m_frontRightActualSpeedPath = "/Chassis/FrontRightModule/ActualState/Speed";
    string m_frontRightActualAnglePath = "/Chassis/FrontRightModule/ActualState/Angle";
    string m_backLeftActualSpeedPath = "/Chassis/BackLeftModule/ActualState/Speed";
    string m_backLeftActualAnglePath = "/Chassis/BackLeftModule/ActualState/Angle";
    string m_backRightActualSpeedPath = "/Chassis/BackRightModule/ActualState/Speed";
    string m_backRightActualAnglePath = "/Chassis/BackRightModule/ActualState/Angle";

    string m_swerveTargetvxPath = "/Chassis/TargetSpeeds/Vx";
    string m_swerveTargetvyPath = "/Chassis/TargetSpeeds/Vy";
    string m_swerveTargetOmegaPath = "/Chassis/TargetSpeeds/Omega";

    string m_swerveActualvxPath = "/Chassis/ActualSpeeds/Vx";
    string m_swerveActualvyPath = "/Chassis/ActualSpeeds/Vy";
    string m_swerveActualOmegaPath = "/Chassis/ActualSpeeds/Omega";

    string m_swerveModuleStateUnits = "m/s, Degrees";
    string m_swerveChassisSpeedUnits = "Vx, Vy, Omega";

    units::time::second_t m_latency = units::time::second_t(0);

protected:
    void LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value);
    void LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value, string units);
    void LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, string value);
    void Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value);
    void Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value);

    void LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value);
    void LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value);

    const double m_doubleTolerance = 0.001;
};
