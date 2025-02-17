
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

protected:
    void LogBoolData(uint64_t timestamp, DragonDataLoggerSignals::BoolSignals signalID, bool value);
    void LogDoubleData(uint64_t timestamp, DragonDataLoggerSignals::DoubleSignals signalID, double value);
    void LogStringData(uint64_t timestamp, DragonDataLoggerSignals::StringSignals signalID, std::string value);
    void Log2DPoseData(uint64_t timestamp, DragonDataLoggerSignals::PoseSingals signalID, frc::Pose2d value);
    void Log3DPoseData(uint64_t timestamp, DragonDataLoggerSignals::PoseSingals signalID, frc::Pose3d value);

    void LogSwerveModuleStateData(uint64_t timestamp, DragonDataLoggerSignals::SwerveStateSingals signalID, frc::SwerveModuleState value);
    void LogChassisSpeedsData(uint64_t timestamp, DragonDataLoggerSignals::ChassisSpeedSignals signalID, frc::ChassisSpeeds value);

    const double m_doubleTolerance = 0.001;
};
