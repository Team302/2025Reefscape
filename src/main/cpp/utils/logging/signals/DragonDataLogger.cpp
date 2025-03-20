
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
using namespace std;

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        SignalLogger::WriteBoolean(magic_enum::enum_name(signalID), value, m_latency);
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value, string units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        SignalLogger::WriteDouble(magic_enum::enum_name(signalID), value, units, m_latency);
    }
}

void DragonDataLogger::LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, std::string value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        SignalLogger::WriteString(magic_enum::enum_name(signalID), value, m_latency);
    }
}
void DragonDataLogger::Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        double x = value.X().value();
        double y = value.Y().value();
        double rot = value.Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        SignalLogger::WriteDoubleArray(magic_enum::enum_name(signalID), pose, m_pose2dUnits, m_latency);
    }
}

void DragonDataLogger::Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        double x = value.ToPose2d().X().value();
        double y = value.ToPose2d().Y().value();
        double rot = value.ToPose2d().Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        SignalLogger::WriteDoubleArray(magic_enum::enum_name(signalID), pose, m_pose2dUnits, m_latency);
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        SignalLogger::WriteDouble(string(magic_enum::enum_name(signalID)) + "/speed", speed, "m/s", m_latency);
        SignalLogger::WriteDouble(string(magic_enum::enum_name(signalID)) + "/angle", angle, "degrees", m_latency);
    }
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        // TODO:  need to compare/store; need to do element by element
        switch (signalID)
        {
        case DragonDataLogger::ChassisSpeedSignals::ACTUAL_SPEEDS:
        {
            double vx = value.vx.value();
            double vy = value.vy.value();
            double omega = value.omega.value();
            SignalLogger::WriteDouble(m_swerveActualvxPath, vx, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveActualvyPath, vy, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveActualOmegaPath, omega, m_swerveChassisSpeedUnits, m_latency);
            break;
        }
        case DragonDataLogger::ChassisSpeedSignals::TARGET_SPEEDS:
        {
            double vx = value.vx.value();
            double vy = value.vy.value();
            double omega = value.omega.value();
            SignalLogger::WriteDouble(m_swerveTargetvxPath, vx, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveTargetvyPath, vy, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveTargetOmegaPath, omega, m_swerveChassisSpeedUnits, m_latency);
            break;
        }
        default:
            break;
        }
    }
}
