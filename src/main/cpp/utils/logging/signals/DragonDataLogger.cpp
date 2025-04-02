
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
        if (m_boolSignalMap.find(signalID) == m_boolSignalMap.end())
        {
            string nameprefix = string(magic_enum::enum_name(signalID));
            char delimiter = '_';
            size_t pos = nameprefix.find_first_of(delimiter);
            if (pos != string::npos)
            {
                nameprefix = nameprefix.substr(0, pos) + "/";
            }
            else
            {
                nameprefix = "";
            }
            m_boolSignalMap.insert(pair<DragonDataLogger::BoolSignals, string>(signalID, nameprefix + string(magic_enum::enum_name(signalID))));
        }

        SignalLogger::WriteBoolean(m_boolSignalMap[signalID], value, m_latency);
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value, string units)
{
    if (m_doubleSignalMap.find(signalID) == m_doubleSignalMap.end())
    {
        string nameprefix = string(magic_enum::enum_name(signalID));
        char delimiter = '_';
        size_t pos = nameprefix.find_first_of(delimiter);
        if (pos != string::npos)
        {
            nameprefix = nameprefix.substr(0, pos) + "/";
        }
        else
        {
            nameprefix = "";
        }
        m_doubleSignalMap.insert(pair<DragonDataLogger::DoubleSignals, string>(signalID, nameprefix + string(magic_enum::enum_name(signalID))));
    }

    SignalLogger::WriteDouble(m_doubleSignalMap[signalID], value, units);
}

void DragonDataLogger::LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, string value)
{
    if (m_stringSignalMap.find(signalID) == m_stringSignalMap.end())
    {
        string nameprefix = string(magic_enum::enum_name(signalID));
        char delimiter = '_';
        size_t pos = nameprefix.find_first_of(delimiter);
        if (pos != string::npos)
        {
            nameprefix = nameprefix.substr(0, pos) + "/";
        }
        else
        {
            nameprefix = "";
        }
        m_stringSignalMap.insert(pair<DragonDataLogger::StringSignals, string>(signalID, nameprefix + string(magic_enum::enum_name(signalID))));
    }
    SignalLogger::WriteString(m_stringSignalMap[signalID], value, m_latency);
}
void DragonDataLogger::Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSignals signalID, frc::Pose2d value)
{
    if (m_poseSignalMap.find(signalID) == m_poseSignalMap.end())
    {
        if (m_poseSignalMap.find(signalID) == m_poseSignalMap.end())
        {
            string nameprefix = string(magic_enum::enum_name(signalID));
            char delimiter = '_';
            size_t pos = nameprefix.find_first_of(delimiter);
            if (pos != string::npos)
            {
                nameprefix = nameprefix.substr(0, pos) + "/";
            }
            else
            {
                nameprefix = "";
            }
            m_poseSignalMap.insert(pair<DragonDataLogger::PoseSignals, string>(signalID, nameprefix + string(magic_enum::enum_name(signalID))));
        }
    }
    double x = value.X().value();
    double y = value.Y().value();
    double rot = value.Rotation().Radians().value();
    vector<double> pose = {x, y, rot};
    SignalLogger::WriteDoubleArray(m_poseSignalMap[signalID], pose, m_pose2dUnits, m_latency);
}

void DragonDataLogger::Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSignals signalID, frc::Pose3d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        if (m_poseSignalMap.find(signalID) == m_poseSignalMap.end())
        {
            string nameprefix = string(magic_enum::enum_name(signalID));
            char delimiter = '_';
            size_t pos = nameprefix.find_first_of(delimiter);
            if (pos != string::npos)
            {
                nameprefix = nameprefix.substr(0, pos) + "/";
            }
            else
            {
                nameprefix = "";
            }
        }
        double x = value.ToPose2d().X().value();
        double y = value.ToPose2d().Y().value();
        double rot = value.ToPose2d().Rotation().Radians().value();
        vector<double> pose = {x, y, rot};
        SignalLogger::WriteDoubleArray(m_poseSignalMap[signalID], pose, m_pose2dUnits, m_latency);
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        if (m_swerveStateSignalMap.find(signalID) == m_swerveStateSignalMap.end())
        {
            string nameprefix = string(magic_enum::enum_name(signalID));
            char delimiter = '_';
            size_t pos = nameprefix.find_first_of(delimiter);
            if (pos != string::npos)
            {
                nameprefix = nameprefix.substr(0, pos) + "/";
            }
            else
            {
                nameprefix = "";
            }
        }
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        SignalLogger::WriteDouble(m_swerveStateSignalMap[signalID] + string(magic_enum::enum_name(signalID)) + "/speed", speed, "m/s", m_latency);
        SignalLogger::WriteDouble(m_swerveStateSignalMap[signalID] + string(magic_enum::enum_name(signalID)) + "/angle", angle, "degrees", m_latency);
    }
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {

        if (m_chassisSpeedSignalMap.find(signalID) == m_chassisSpeedSignalMap.end())
        {
            string nameprefix = string(magic_enum::enum_name(signalID));
            char delimiter = '_';
            size_t pos = nameprefix.find_first_of(delimiter);
            if (pos != string::npos)
            {
                nameprefix = nameprefix.substr(0, pos) + "/";
            }
            else
            {
                nameprefix = "";
            }
        }
        double vx = value.vx.value();
        double vy = value.vy.value();
        double omega = value.omega.value();
        SignalLogger::WriteDouble(m_chassisSpeedSignalMap[signalID] + "/vx", vx, m_swerveChassisSpeedUnits, m_latency);
        SignalLogger::WriteDouble(m_chassisSpeedSignalMap[signalID] + string(magic_enum::enum_name(signalID)) + "/vy", vy, m_swerveChassisSpeedUnits, m_latency);
        SignalLogger::WriteDouble(m_chassisSpeedSignalMap[signalID] + string(magic_enum::enum_name(signalID)) + "/omega", omega, m_swerveChassisSpeedUnits, m_latency);
    }
}
