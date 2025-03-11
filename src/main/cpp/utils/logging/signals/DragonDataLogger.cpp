
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

#include "ctre/phoenix6/SignalLogger.hpp"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

using ctre::phoenix6::SignalLogger;

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::BoolSignals::IS_BROWNOUT:
            SignalLogger::WriteBoolean(m_brownOutPath, value, m_latency);
            break;
        default:
            break;
        }
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES:
            SignalLogger::WriteDouble(m_storedHeadingPath, value, m_storedHeadingUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::CHASSIS_YAW_DEGREES:
            SignalLogger::WriteDouble(m_ChassisYawPath, value, m_ChassisYawUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_VOLTAGE:
            SignalLogger::WriteDouble(m_electricalEnergyPath, value, m_electricalVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_CURRENT:
            SignalLogger::WriteDouble(m_electricalCurrentPath, value, m_electricalCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_ENERGY:
            SignalLogger::WriteDouble(m_electricalEnergyPath, value, m_electricalEnergyUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_POWER:
            SignalLogger::WriteDouble(m_electricalPowerPath, value, m_electricalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TV_1:
            // SignalLogger::WriteDouble(m_storedHeadingPath, value, m_storedHeadingUnits, m_latency);

            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TX_1:
            dataMgr->m_tx.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TY_1:
            dataMgr->m_ty.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_FIDUCIAL_ID_1:
            SignalLogger::WriteDouble(m_fiducialPath, value, m_fiducialUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::BATTERY_VOLTAGE:
            SignalLogger::WriteDouble(m_batteryVoltagePath, value, m_batteryVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::BROWNOUT_VOLTAGE:
            SignalLogger::WriteDouble(m_brownoutVoltagePath, value, m_brownoutVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::INPUT_VOLTAGE:
            SignalLogger::WriteDouble(m_inputVoltagePath, value, m_inputVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::INPUT_CURRENT:
            SignalLogger::WriteDouble(m_inputCurrentPath, value, m_inputCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::CPU_TEMP:
            SignalLogger::WriteDouble(m_cpuTempPath, value, m_cpuTempUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_POWER:
            SignalLogger::WriteDouble(m_lfSteerPowerPath, value, m_lfSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_ENERGY:
            SignalLogger::WriteDouble(m_lfSteerEnergyPath, value, m_lfSteerEnergyUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_TOTALPOWER:
            dataMgr->m_lfSteerTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_WATT_HOURS:
            dataMgr->m_lfSteerWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_POWER:
            dataMgr->m_lfDrivePower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_ENERGY:
            dataMgr->m_lfDriveEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER:
            dataMgr->m_lfDriveTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS:
            dataMgr->m_lfDriveWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_POWER:
            dataMgr->m_rfSteerPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_ENERGY:
            dataMgr->m_rfSteerEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_TOTALPOWER:
            dataMgr->m_rfSteerTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_WATT_HOURS:
            dataMgr->m_rfSteerWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_POWER:
            dataMgr->m_rfDrivePower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_ENERGY:
            dataMgr->m_rfDriveEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER:
            dataMgr->m_rfDriveTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS:
            dataMgr->m_rfDriveWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_POWER:
            dataMgr->m_lbSteerPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_ENERGY:
            dataMgr->m_lbSteerEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_TOTALPOWER:
            dataMgr->m_lbSteerTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_WATT_HOURS:
            dataMgr->m_lbSteerWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_POWER:
            dataMgr->m_lbDrivePower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_ENERGY:
            dataMgr->m_lbDriveEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_TOTALPOWER:
            dataMgr->m_lbDriveTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_WATT_HOURS:
            dataMgr->m_lbDriveWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_POWER:
            dataMgr->m_rbSteerPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_ENERGY:
            dataMgr->m_rbSteerEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_TOTALPOWER:
            dataMgr->m_rbSteerTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_WATT_HOURS:
            dataMgr->m_rbSteerWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_POWER:
            dataMgr->m_rbDrivePower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_ENERGY:
            dataMgr->m_rbDriveEnergy.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER:
            dataMgr->m_rbDriveTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS:
            dataMgr->m_rbDriveWattHours.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_TOTAL_ENERGY:
            dataMgr->m_swerveChassisTotalPower.Update(value, timestamp);
            break;

        case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_WATT_HOURS:
            dataMgr->m_swerveChassisWattHours.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, std::string value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::StringSignals::CHASSIS_DRIVE_STATE:
            dataMgr->m_driveState.Update(value, timestamp);
            break;

        case DragonDataLogger::StringSignals::CHASSIS_HEADING_STATE:
            dataMgr->m_headingState.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}
void DragonDataLogger::Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_POSE2D:
            dataMgr->m_pose2d.Update(value, timestamp);
            break;
        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE:
            dataMgr->m_visionLeftReefBranchPose.Update(value, timestamp);
            break;
        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE:
            dataMgr->m_visionRightReefBranchPose.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE:
            dataMgr->m_visionCoralStationPose.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE:
            dataMgr->m_odometryLeftReefBranchPose.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE:
            dataMgr->m_odometryRightReefBranchPose.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE:
            dataMgr->m_odometryCoralStationPose.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D:
            dataMgr->m_pose3dLimelight.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT2_POSE3D:
            dataMgr->m_pose3dLimelight2.Update(value, timestamp);
            break;

        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE3D:
            dataMgr->m_pose3dQuest.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_FRONT_STATE:
            dataMgr->m_frontLeftTarget.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_BACK_STATE:
            dataMgr->m_backLeftTarget.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE:
            dataMgr->m_frontRightTarget.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_BACK_STATE:
            dataMgr->m_backRightTarget.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE:
            dataMgr->m_frontLeftActual.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE:
            dataMgr->m_backLeftActual.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE:
            dataMgr->m_frontRightActual.Update(value, timestamp);
            break;

        case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE:
            dataMgr->m_backRightActual.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeeddataMgr signalID, frc::ChassisSpeeds value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        // TODO:  need to compare/store; need to do element by element
        switch (signalID)
        {
        case DragonDataLogger::ChassisSpeeddataMgr::ACTUAL_SPEEDS:
            dataMgr->m_actualSpeeds.Update(value, timestamp);
            break;

        case DragonDataLogger::ChassisSpeeddataMgr::TARGET_SPEEDS:
            dataMgr->m_targetSpeeds.Update(value, timestamp);
            break;

        default:
            break;
        }
    }
}
