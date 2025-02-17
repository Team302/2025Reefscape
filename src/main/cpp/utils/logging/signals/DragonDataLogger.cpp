
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

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(units::time::second_t timestamp, DragonDataLoggerSignals::BoolSignals signalID, bool value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
    }
}

void DragonDataLogger::LogDoubleData(units::time::second_t timestamp, DragonDataLoggerSignals::DoubleSignals signalID, double value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES:
            signals->m_storedHeading.Update(value, timestamp.value());
            break;
        case DragonDataLoggerSignals::DoubleSignals::CHASSIS_YAW_DEGREES:
            signals->m_chassisYaw.Update(value, timestamp.value());
            break;
        case DragonDataLoggerSignals::DoubleSignals::ELECTRICAL_VOLTAGE:
            signals->m_electricalVoltage.Update(value, timestamp.value());
            break;
        case DragonDataLoggerSignals::DoubleSignals::ELECTRICAL_CURRENT:
            signals->m_electricalCurrent.Update(value, timestamp.value());
            break;
        case DragonDataLoggerSignals::DoubleSignals::ELECTRICAL_ENERGY:
            signals->m_electricalEnergy.Update(value, timestamp.value());
            break;
        case DragonDataLoggerSignals::DoubleSignals::ELECTRICAL_POWER:
            signals->m_electricalPower.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::DoubleSignals::LIMELIGHT_TV_1:
            signals->m_tv.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::DoubleSignals::LIMELIGHT_TX_1:
            signals->m_tx.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::DoubleSignals::LIMELIGHT_TY_1:
            signals->m_ty.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::DoubleSignals::LIMELIGHT_FIDUCIAL_ID_1:
            signals->m_Fiducial.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogStringData(units::time::second_t timestamp, DragonDataLoggerSignals::StringSignals signalID, std::string value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::StringSignals::CHASSIS_DRIVE_STATE:
            signals->m_driveState.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::StringSignals::CHASSIS_HEADING_STATE:
            signals->m_headingState.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}
void DragonDataLogger::Log2DPoseData(units::time::second_t timestamp, DragonDataLoggerSignals::PoseSingals signalID, frc::Pose2d value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_POSE2D:
            signals->m_pose2d.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::Log3DPoseData(units::time::second_t timestamp, DragonDataLoggerSignals::PoseSingals signalID, frc::Pose3d value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D:
            signals->m_pose3dLimelight.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_LIMELIGHT2_POSE3D:
            signals->m_pose3dLimelight2.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_QUEST_POSE3D:
            signals->m_pose3dQuest.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogSwerveModuleStateData(units::time::second_t timestamp, DragonDataLoggerSignals::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_FRONT_STATE:
            signals->m_frontLeftTarget.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_BACK_STATE:
            signals->m_backLeftTarget.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE:
            signals->m_frontRightTarget.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_BACK_STATE:
            signals->m_backRightTarget.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE:
            signals->m_frontLeftActual.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE:
            signals->m_backLeftActual.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE:
            signals->m_frontRightActual.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE:
            signals->m_backRightActual.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogChassisSpeedsData(units::time::second_t timestamp, DragonDataLoggerSignals::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        // TODO:  need to compare/store; need to do element by element
        switch (signalID)
        {
        case DragonDataLoggerSignals::ChassisSpeedSignals::ACTUAL_SPEEDS:
            signals->m_actualSpeeds.Update(value, timestamp.value());
            break;

        case DragonDataLoggerSignals::ChassisSpeedSignals::TARGET_SPEEDS:
            signals->m_targetSpeeds.Update(value, timestamp.value());
            break;

        default:
            break;
        }
    }
}
