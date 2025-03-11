
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
        ELECTRICAL_ENERGY,
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
        LEFT_FRONT_SWERVE_STEER_ENERGY,
        LEFT_FRONT_SWERVE_STEER_TOTALPOWER,
        LEFT_FRONT_SWERVE_STEER_WATT_HOURS,
        LEFT_FRONT_SWERVE_DRIVE_POWER,
        LEFT_FRONT_SWERVE_DRIVE_ENERGY,
        LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_FRONT_SWERVE_STEER_POWER,
        RIGHT_FRONT_SWERVE_STEER_ENERGY,
        RIGHT_FRONT_SWERVE_STEER_TOTALPOWER,
        RIGHT_FRONT_SWERVE_STEER_WATT_HOURS,
        RIGHT_FRONT_SWERVE_DRIVE_POWER,
        RIGHT_FRONT_SWERVE_DRIVE_ENERGY,
        RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        LEFT_BACK_SWERVE_STEER_POWER,
        LEFT_BACK_SWERVE_STEER_ENERGY,
        LEFT_BACK_SWERVE_STEER_TOTALPOWER,
        LEFT_BACK_SWERVE_STEER_WATT_HOURS,
        LEFT_BACK_SWERVE_DRIVE_POWER,
        LEFT_BACK_SWERVE_DRIVE_ENERGY,
        LEFT_BACK_SWERVE_DRIVE_TOTALPOWER,
        LEFT_BACK_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_BACK_SWERVE_STEER_POWER,
        RIGHT_BACK_SWERVE_STEER_ENERGY,
        RIGHT_BACK_SWERVE_STEER_TOTALPOWER,
        RIGHT_BACK_SWERVE_STEER_WATT_HOURS,
        RIGHT_BACK_SWERVE_DRIVE_POWER,
        RIGHT_BACK_SWERVE_DRIVE_ENERGY,
        RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS,
        SWERVE_CHASSIS_TOTAL_ENERGY,
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

    std::string m_brownOutPath = "/RoboRio/IsBrownOut";

    std::string m_storedHeadingPath = "/Chassis/StoredHeading";
    std::string m_storedHeadingUnits = "Degrees";

    std::string m_ChassisYawPath = "/Chassis/Yaw";
    std::string m_ChassisYawUnits = "Degrees";
    std::string m_electricalVoltagePath = "/Electrical/Voltage";

    std::string m_electricalVoltageUnits = "";
    std::string m_electricalCurrentPath = "";
    std::string m_electricalCurrentUnits = "";
    std::string m_electricalEnergyPath = "";
    std::string m_electricalEnergyUnits = "";
    std::string m_electricalPowerPath = "";
    std::string m_electricalPowerUnits = "";

    std::string m_txPath = "";
    std::string m_txUnits = "";
    std::string m_tyPath = "";
    std::string m_tyUnits = "";
    std::string m_tvPath = "";
    std::string m_tvUnits = "";
    std::string m_fiducialPath = "";
    std::string m_fiducialUnits = "";

    // RIO

    std::string m_batteryVoltagePath = "/RoboRio/BatteryVoltage";
    std::string m_batteryVoltageUnits = "Volts";
    std::string m_brownoutVoltagePath = "/RoboRio/BrownoutVoltage";
    std::string m_brownoutVoltageUnits = "Volts";
    std::string m_inputVoltagePath = "/RoboRio/InputVoltage";
    std::string m_inputVoltageUnits = "Volts";
    std::string m_inputCurrentPath = "/RoboRio/InputCurrent";
    std::string m_inputCurrentUnits = "Amps?";
    std::string m_cpuTempPath = "/RoboRio/CPUTemp";
    std::string m_cpuTempUnits = "Degrees C";

    std::string m_lfSteerPowerPath = "/Chassis/FrontLeftModule/Steer/Power";
    std::string m_lfSteerPowerUnits = "";
    std::string m_lfSteerEnergyPath = "/Chassis/FrontLeftModule/Steer/Energy";
    std::string m_lfSteerEnergyUnits = "";
    std::string m_lfSteerTotalPowerPath = "/Chassis/FrontLeftModule/Steer/TotalPower";
    std::string m_lfSteerTotalPowerUnits = "";
    std::string m_lfSteerWattHoursPath = "/Chassis/FrontLeftModule/Steer/WattHours";
    std::string m_lfSteerWattHoursUnits = "";

    std::string m_lfDrivePowerPath = "/Chassis/FrontLeftModule/Drive/Power";
    std::string m_lfDrivePowerUnits = "";
    std::string m_lfDriveEnergyPath = "/Chassis/FrontLeftModule/Drive/Energy";
    std::string m_lfDriveEnergyUnits = "";
    std::string m_lfDriveTotalPowerPath = "/Chassis/FrontLeftModule/Drive/TotalPower";
    std::string m_lfDriveTotalPowerUnits = "";
    std::string m_lfDriveWattHoursPath = "/Chassis/FrontLeftModule/Drive/WattHours";
    std::string m_lfDriveWattHoursUnits = "";

    std::string m_rfSteerPowerPath = "/Chassis/FrontRightModule/Steer/Power";
    std::string m_rfSteerPowerUnits = "";
    std::string m_rfSteerEnergyPath = "/Chassis/FrontRightModule/Steer/Energy";
    std::string m_rfSteerEnergyUnits = "";
    std::string m_rfSteerTotalPowerPath = "/Chassis/FrontRightModule/Steer/TotalPower";
    std::string m_rfSteerTotalPowerUnits = "";
    std::string m_rfSteerWattHoursPath = "/Chassis/FrontRightModule/Steer/WattHours";
    std::string m_rfSteerWattHoursUnits = "";

    std::string m_rfDrivePowerPath = "/Chassis/FrontRightModule/Drive/Power";
    std::string m_rfDrivePowerUnits = "";
    std::string m_rfDriveEnergyPath = "/Chassis/FrontRightModule/Drive/Energy";
    std::string m_rfDriveEnergyUnits = "";
    std::string m_rfDriveTotalPowerPath = "/Chassis/FrontRightModule/Drive/TotalPower";
    std::string m_rfDriveTotalPowerUnits = "";
    std::string m_rfDriveWattHoursPath = "/Chassis/FrontRightModule/Drive/WattHours";
    std::string m_rfDriveWattHoursUnits = "";

    std::string m_lbSteerPowerPath = "/Chassis/BackLeftModule/Steer/Power";
    std::string m_lbSteerPowerUnits = "";
    std::string m_lbSteerEnergyPath = "/Chassis/BackLeftModule/Steer/Energy";
    std::string m_lbSteerEnergyUnits = "";
    std::string m_lbSteerTotalPowerPath = "/Chassis/BackLeftModule/Steer/TotalPower";
    std::string m_lbSteerTotalPowerUnits = "";
    std::string m_lbSteerWattHoursPath = "/Chassis/BackLeftModule/Steer/WattHours";
    std::string m_lbSteerWattHoursUnits = "";

    std::string m_lbDrivePowerPath = "/Chassis/BackLeftModule/Drive/Power";
    std::string m_lbDrivePowerUnits = "";
    std::string m_lbDriveEnergyPath = "/Chassis/BackLeftModule/Drive/Energy";
    std::string m_lbDriveEnergyUnits = "";
    std::string m_lbDriveTotalPowerPath = "/Chassis/BackLeftModule/Drive/TotalPower";
    std::string m_lbDriveTotalPowerUnits = "";
    std::string m_lbDriveWattHoursPath = "/Chassis/BackLeftModule/Drive/WattHours";
    std::string m_lbDriveWattHoursUnits = "";

    std::string m_rbSteerPowerPath = "/Chassis/BackRightModule/Steer/Power";
    std::string m_rbSteerPowerUnits = "";
    std::string m_rbSteerEnergyPath = "/Chassis/BackRightModule/Steer/Energy";
    std::string m_rbSteerEnergyUnits = "";
    std::string m_rbSteerTotalPowerPath = "";
    std::string m_rbSteerTotalPowerUnits = "/Chassis/BackRightModule/Steer/TotalPower";
    std::string m_rbSteerWattHoursPath = "";
    std::string m_rbSteerWattHoursUnits = "/Chassis/BackRightModule/Steer/WattHours";

    std::string m_rbDrivePowerPath = "/Chassis/BackRightModule/Drive/Power";
    std::string m_rbDrivePowerUnits = "";
    std::string m_rbDriveEnergyPath = "/Chassis/BackRightModule/Drive/Energy";
    std::string m_rbDriveEnergyUnits = "";
    std::string m_rbDriveTotalPowerPath = "/Chassis/BackRightModule/Drive/TotalPower";
    std::string m_rbDriveTotalPowerUnits = "";
    std::string m_rbDriveWattHoursPath = "/Chassis/BackRightModule/Drive/WattHours";
    std::string m_rbDriveWattHoursUnits = "";

    std::string m_swerveChassisTotalPowerPath = "/Chassis/TotalPower";
    std::string m_swerveChassisTotalPowerUnits = "";
    std::string m_swerveChassisWattHoursPath = "/Chassis/WattHours";
    std::string m_swerveChassisWattHoursUnits = "";
    std::string m_headingStatePath = "/Chassis/HeadingState";
    std::string m_driveStatePath = "/Chassis/DriveState";

    units::time::second_t m_latency = units::time::second_t(0);

protected:
    void LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value);
    void LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value);
    void LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, std::string value);
    void Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value);
    void Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value);

    void LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value);
    void LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value);

    const double m_doubleTolerance = 0.001;
};
