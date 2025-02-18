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
// FRC includes
#include "units/length.h"

// Team 302 includes
#include "auton/PrimitiveEnums.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "auton/PrimitiveParams.h"

class DriveToHelper
{
public:
    DriveToHelper(SwerveChassis *m_chassis, ChassisMovement m_moveInfo);
    ~DriveToHelper() = default;

    void Init(UPDATE_OPTION pathUpdateOption);
    void Run();
    bool IsDone();

private:
    TrajectoryDrivePathPlanner *m_driveTo;
    SwerveChassis *m_chassis;
    const units::length::meter_t m_distanceThreshold = units::length::meter_t(1.0);
    bool m_isVisionDrive = false;

    ChassisMovement m_moveInfo;
    UPDATE_OPTION m_pathUpdateOption;

    PrimitiveParams::VISION_ALIGNMENT m_visionAlignment;
};