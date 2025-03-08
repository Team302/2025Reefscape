
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
    wpi::log::BooleanLogEntry m_isBrownOut;

    wpi::log::DoubleLogEntry m_storedHeading;
    wpi::log::DoubleLogEntry m_chassisYaw;

    wpi::log::DoubleLogEntry m_electricalVoltage;
    wpi::log::DoubleLogEntry m_electricalCurrent;
    wpi::log::DoubleLogEntry m_electricalEnergy;
    wpi::log::DoubleLogEntry m_electricalPower;

    wpi::log::DoubleLogEntry m_tx;
    wpi::log::DoubleLogEntry m_ty;
    wpi::log::DoubleLogEntry m_tv;
    wpi::log::DoubleLogEntry m_fiducial;

    wpi::log::DoubleLogEntry m_batteryVoltage;
    wpi::log::DoubleLogEntry m_brownoutVoltage;
    wpi::log::DoubleLogEntry m_inputVoltage;
    wpi::log::DoubleLogEntry m_inputCurrent;
    wpi::log::DoubleLogEntry m_cpuTemp;

    wpi::log::DoubleLogEntry m_lfSteerPower;
    wpi::log::DoubleLogEntry m_lfSteerEnergy;
    wpi::log::DoubleLogEntry m_lfSteerTotalPower;
    wpi::log::DoubleLogEntry m_lfSteerWattHours;

    wpi::log::DoubleLogEntry m_lfDrivePower;
    wpi::log::DoubleLogEntry m_lfDriveEnergy;
    wpi::log::DoubleLogEntry m_lfDriveTotalPower;
    wpi::log::DoubleLogEntry m_lfDriveWattHours;

    wpi::log::DoubleLogEntry m_rfSteerPower;
    wpi::log::DoubleLogEntry m_rfSteerEnergy;
    wpi::log::DoubleLogEntry m_rfSteerTotalPower;
    wpi::log::DoubleLogEntry m_rfSteerWattHours;

    wpi::log::DoubleLogEntry m_rfDrivePower;
    wpi::log::DoubleLogEntry m_rfDriveEnergy;
    wpi::log::DoubleLogEntry m_rfDriveTotalPower;
    wpi::log::DoubleLogEntry m_rfDriveWattHours;

    wpi::log::DoubleLogEntry m_lbSteerPower;
    wpi::log::DoubleLogEntry m_lbSteerEnergy;
    wpi::log::DoubleLogEntry m_lbSteerTotalPower;
    wpi::log::DoubleLogEntry m_lbSteerWattHours;

    wpi::log::DoubleLogEntry m_lbDrivePower;
    wpi::log::DoubleLogEntry m_lbDriveEnergy;
    wpi::log::DoubleLogEntry m_lbDriveTotalPower;
    wpi::log::DoubleLogEntry m_lbDriveWattHours;

    wpi::log::DoubleLogEntry m_rbSteerPower;
    wpi::log::DoubleLogEntry m_rbSteerEnergy;
    wpi::log::DoubleLogEntry m_rbSteerTotalPower;
    wpi::log::DoubleLogEntry m_rbSteerWattHours;

    wpi::log::DoubleLogEntry m_rbDrivePower;
    wpi::log::DoubleLogEntry m_rbDriveEnergy;
    wpi::log::DoubleLogEntry m_rbDriveTotalPower;
    wpi::log::DoubleLogEntry m_rbDriveWattHours;

    wpi::log::DoubleLogEntry m_swerveChassisTotalPower;
    wpi::log::DoubleLogEntry m_swerveChassisWattHours;

    wpi::log::StringLogEntry m_headingState;
    wpi::log::StringLogEntry m_driveState;

    wpi::log::StructLogEntry<frc::Pose2d> m_pose2d;
    wpi::log::StructLogEntry<frc::Pose2d> m_visionLeftReefBranchPose;
    wpi::log::StructLogEntry<frc::Pose2d> m_visionRightReefBranchPose;
    wpi::log::StructLogEntry<frc::Pose2d> m_visionCoralStationPose;
    wpi::log::StructLogEntry<frc::Pose2d> m_odometryLeftReefBranchPose;
    wpi::log::StructLogEntry<frc::Pose2d> m_odometryRightReefBranchPose;
    wpi::log::StructLogEntry<frc::Pose2d> m_odometryCoralStationPose;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dLimelight;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dLimelight2;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dQuest;

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontLeftTarget;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontRightTarget;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backLeftTarget;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backRightTarget;

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontLeftActual;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontRightActual;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backLeftActual;
    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backRightActual;

    wpi::log::StructLogEntry<frc::ChassisSpeeds> m_targetSpeeds;
    wpi::log::StructLogEntry<frc::ChassisSpeeds> m_actualSpeeds;

    DragonDataLoggerSignals();
    virtual ~DragonDataLoggerSignals() = delete;

    static DragonDataLoggerSignals *m_instance;
};
