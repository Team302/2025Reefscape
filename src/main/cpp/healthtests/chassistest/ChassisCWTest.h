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
// OR OTER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#pragma once
#include "chassis/HolonomicDrive.h"
#include "auton/drivePrimitives/DriveStop.h"
#include "frc/Timer.h"
#include "healthtests/DragonTestCase.h"
#include "chassis/ChassisMovement.h"
#include "auton/drivePrimitives/DrivePathPlanner.h"
#include "utils/logging/DragonDataLogger.h"
#include "frc/geometry/Pose2d.h"

class ChassisCWTest : public DragonTestCase, public DragonDataLogger
{
    ChassisCWTest();
    ~ChassisCWTest() = default;

    void SetUp() override;
    bool Run() override;
    void CompareAndReport() override;

private:
    SwerveChassis *m_swerve;
    frc::Timer *m_timer;
    ChassisMovement m_moveInfo;
    const units::second_t m_maxtime = units::second_t(10.0);
    DrivePathPlanner *m_drivePathPlanner;

    frc::Pose2d m_orignalPose;
};
