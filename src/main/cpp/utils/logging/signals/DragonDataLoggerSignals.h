
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

// #include <array>
#include <map>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "wpi/DataLog.h"

class DragonDataLoggerSignals
{
    friend class DragonDataLogger;

public:
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
        CURRENT_CHASSIS_QUEST_POSE3D,
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

    static DragonDataLoggerSignals *GetInstance();

private:
    // initialize these signals in the constructor

    std::string m_brownOutPath = "";

    std::string m_storedHeadingPath = "";
    std::string m_ChassisYawPath = "";

    std::string m_electricalVoltagePath = "";
    std::string m_electricalCurrentPath = "";
    std::string m_electricalEnergyPath = "";
    std::string m_electricalPowerPath = "";

    std::string m_txPath = "";
    std::string m_tyPath = "";
    std::string m_tvPath = "";
    std::string m_fiducialPath = "";

    std::string m_batteryVoltagePath = "";
    std::string m_brownoutVoltagePath = "";
    std::string m_inputVoltagePath = "";
    std::string m_inputCurrentPath = "";
    std::string m_cpuTempPath = "";

    std::string m_lfSteerPowerPath = "";
    std::string m_lfSteerEnergyPath = "";
    std::string m_lfSteerTotalPowerPath = "";
    std::string m_lfSteerWattHoursPath = "";

    std::string m_lfDrivePowerPath = "";
    std::string m_lfDriveEnergyPath = "";
    std::string m_lfDriveTotalPowerPath = "";
    std::string m_lfDriveWattHoursPath = "";
    std::string m_rfSteerPowerPath = "";
    std::string m_rfSteerEnergyPath = "";
    std::string m_rfSteerTotalPowerPath = "";
    std::string m_rfSteerWattHoursPath = "";
    std::string m_rfDrivePowerPath = "";
    std::string m_rfDriveEnergyPath = "";
    std::string m_rfDriveTotalPowerPath = "";
    std::string m_rfDriveWattHoursPath = "";

    std::string m_lbSteerPowerPath = "";
    std::string m_lbSteerEnergyPath = "";
    std::string m_lbSteerTotalPowerPath = "";
    std::string m_lbSteerWattHoursPath = "";
    std::string m_lbDrivePowerPath = "";
    std::string m_lbDriveEnergyPath = "";
    std::string m_lbDriveTotalPowerPath = "";
    std::string m_lbDriveWattHoursPath = "";
    std::string m_rbSteerPowerPath = "";
    std::string m_rbSteerEnergyPath = "";
    std::string m_rbSteerTotalPowerPath = "";
    std::string m_rbSteerWattHoursPath = "";
    std::string m_rbDrivePowerPath = "";
    std::string m_rbDriveEnergyPath = "";
    std::string m_rbDriveTotalPowerPath = "";
    std::string m_rbDriveWattHoursPath = "";
    std::string m_swerveChassisTotalPowerPath = "";
    std::string m_swerveChassisWattHoursPath = "";
    std::string m_headingStatePath = "";
    std::string m_driveStatePath = "";

    DragonDataLoggerSignals();
    virtual ~DragonDataLoggerSignals() = delete;

    static DragonDataLoggerSignals *m_instance;
};
