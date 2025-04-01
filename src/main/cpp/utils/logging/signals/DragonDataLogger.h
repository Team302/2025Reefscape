
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
        RIO_IS_BROWNOUT,

        TALE_CORAL_IN_SENSOR,
        TALE_CORAL_OUT_SENSOR,
        TALE_ALGAE_SENSOR
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

        RIO_BATTERY_VOLTAGE,
        RIO_BROWNOUT_VOLTAGE,
        RIO_INPUT_VOLTAGE,
        RIO_INPUT_CURRENT,
        RIO_CPU_TEMP,

        CHASSIS_LEFT_FRONT_SWERVE_STEER_POWER,
        CHASSIS_LEFT_FRONT_SWERVE_STEER_CURRENT,
        CHASSIS_LEFT_FRONT_SWERVE_STEER_TOTALPOWER,
        CHASSIS_LEFT_FRONT_SWERVE_STEER_WATT_HOURS,
        CHASSIS_LEFT_FRONT_SWERVE_DRIVE_POWER,
        CHASSIS_LEFT_FRONT_SWERVE_DRIVE_CURRENT,
        CHASSIS_LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        CHASSIS_LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        CHASSIS_RIGHT_FRONT_SWERVE_STEER_POWER,
        CHASSIS_RIGHT_FRONT_SWERVE_STEER_CURRENT,
        CHASSIS_RIGHT_FRONT_SWERVE_STEER_TOTALPOWER,
        CHASSIS_RIGHT_FRONT_SWERVE_STEER_WATT_HOURS,
        CHASSIS_RIGHT_FRONT_SWERVE_DRIVE_POWER,
        CHASSIS_RIGHT_FRONT_SWERVE_DRIVE_CURRENT,
        CHASSIS_RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        CHASSIS_RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        CHASSIS_LEFT_BACK_SWERVE_STEER_POWER,
        CHASSIS_LEFT_BACK_SWERVE_STEER_CURRENT,
        CHASSIS_LEFT_BACK_SWERVE_STEER_TOTALPOWER,
        CHASSIS_LEFT_BACK_SWERVE_STEER_WATT_HOURS,
        CHASSIS_LEFT_BACK_SWERVE_DRIVE_POWER,
        CHASSIS_LEFT_BACK_SWERVE_DRIVE_CURRENT,
        CHASSIS_LEFT_BACK_SWERVE_DRIVE_TOTALPOWER,
        CHASSIS_LEFT_BACK_SWERVE_DRIVE_WATT_HOURS,
        CHASSIS_RIGHT_BACK_SWERVE_STEER_POWER,
        CHASSIS_RIGHT_BACK_SWERVE_STEER_CURRENT,
        CHASSIS_RIGHT_BACK_SWERVE_STEER_TOTALPOWER,
        CHASSIS_RIGHT_BACK_SWERVE_STEER_WATT_HOURS,
        CHASSIS_RIGHT_BACK_SWERVE_DRIVE_POWER,
        CHASSIS_RIGHT_BACK_SWERVE_DRIVE_CURRENT,
        CHASSIS_RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER,
        CHASSIS_RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS,
        CHASSIS_SWERVE_TOTAL_POWER,
        CHASSIS_SWERVE_WATT_HOURS,

        TALE_ARM_ANGLE,
        TALE_ARM_TRAGET_ANGLE,
        TALE_ELEVATOR_LEADER_HEIGHT,
        TALE_ELEVATOR_LEADER_TARGET,

        TALE_ARM_POWER,
        TALE_ARM_CURRENT,
        TALE_ELEVATOR_LEADER_POWER,
        TALE_ELEVATOR_LEADER_CURRENT,
        TALE_ELEVATOR_FOLLOWER_POWER,
        TALE_ELEVATOR_FOLLOWER_CURRENT,
        TALE_ALGAE_POWER,
        TALE_ALGAE_CURRENT,
        TALE_CORAL_POWER,
        TALE_CORAL_CURRENT,
        TALE_TOTAL_POWER,
        TALE_TOTAL_ENERGY,

        CLIMBER_ANGLE,
        CLIMBER_TARGET,
        CLIMBER_ENERGY,
        CLIMBER_CURRENT,
        CLIMBER_TOTAL_ENERGY,
        CLIMBER_TOTAL_POWER
    };

    enum StringSignals
    {
        CHASSIS_HEADING_STATE,
        CHASSIS_DRIVE_STATE,
        AUTON_PATH_NAME,

        TALE_STATE,

        CLIMBER_STATE
    };

    enum PoseSignals
    {
        POSE2D_CURRENT_CHASSIS_POSE2D,
        POSE2D_CURRENT_CHASSIS_LIMELIGHT_POSE3D,
        POSE2D_CURRENT_CHASSIS_LIMELIGHT2_POSE3D,
        POSE2D_CURRENT_CHASSIS_QUEST_POSE2D,
        POSE2D_VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        POSE2D_VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        POSE2D_VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE,
        POSE2D_ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        POSE2D_ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        POSE2D_ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE

    };

    enum ChassisSpeedSignals
    {
        CHASSIS_TARGET_SPEEDS,
        CHASSIS_ACTUAL_SPEEDS
    };

    enum SwerveStateSingals
    {
        CHASSIS_TARGET_LEFT_FRONT_STATE,
        CHASSIS_TARGET_RIGHT_FRONT_STATE,
        CHASSIS_TARGET_LEFT_BACK_STATE,
        CHASSIS_TARGET_RIGHT_BACK_STATE,
        CHASSIS_ACTUAL_LEFT_FRONT_STATE,
        CHASSIS_ACTUAL_RIGHT_FRONT_STATE,
        CHASSIS_ACTUAL_LEFT_BACK_STATE,
        CHASSIS_ACTUAL_RIGHT_BACK_STATE
    };

    // initialize these signals in the constructor

    string m_pose2dUnits = "X, Y, Rotation";
    string m_swerveModuleStateUnits = "m/s, Degrees";
    string m_swerveChassisSpeedUnits = "Vx, Vy, Omega";

    units::time::second_t m_latency = units::time::second_t(0);

protected:
    void LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value);
    void LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value, string units);
    void LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, string value);
    void Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSignals signalID, frc::Pose2d value);
    void Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSignals signalID, frc::Pose3d value);

    void LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value);
    void LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value);

    const double m_doubleTolerance = 0.001;

private:
    std::unordered_map<BoolSignals, string> m_boolSignalMap;
    std::unordered_map<DoubleSignals, string> m_doubleSignalMap;
    std::unordered_map<StringSignals, string> m_stringSignalMap;
    std::unordered_map<PoseSignals, string> m_poseSignalMap;
    std::unordered_map<ChassisSpeedSignals, string> m_chassisSpeedSignalMap;
    std::unordered_map<SwerveStateSingals, string> m_swerveStateSignalMap;
};
