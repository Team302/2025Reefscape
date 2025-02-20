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
// C++ Includes
#include <algorithm>
#include <string>
#include <iostream>

#include "auton/drivePrimitives/DriveStop.h"
#include "chassis/HolonomicDrive.h"
#include "auton/drivePrimitives/DriveStop.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "ChassisBackwardTest.h"
#include "teleopcontrol/TeleopControl.h"

ChassisBackwardTest::ChassisBackwardTest() : DragonTestCase(std::string("drive"), std::string("forwards"))
{
    m_timer = new frc::Timer();
    m_swerve = ChassisConfigMgr::GetInstance()->GetCurrentConfig() != nullptr ? ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis() : nullptr;
}
void ChassisBackwardTest::SetUp()
{
    m_timer->Reset();
    m_timer->Start();
    m_moveInfo.rawX = 0.0;
    m_moveInfo.rawY = 0.0;
    m_moveInfo.rawOmega = 0.0;
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    m_moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    m_moveInfo.pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    m_moveInfo.centerOfRotationOffset = frc::Translation2d();
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    m_moveInfo.noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    m_moveInfo.yawAngle = units::angle::degree_t(0.0);
    m_moveInfo.checkTipping = false;
    m_moveInfo.tippingTolerance = units::angle::degree_t(5.0);
    m_moveInfo.tippingCorrection = -0.1;
    m_moveInfo.targetPose = frc::Pose2d();
    m_moveInfo.rawX = 1.0;
    m_moveInfo.rawY = 0;
    m_moveInfo.rawOmega = 0;
}
bool ChassisBackwardTest::Run()
{
    bool countinueToRun = m_timer->HasElapsed(m_maxtime);
    if (!countinueToRun)
    {
        m_moveInfo.rawX = 0.0;
    }
    m_swerve->Drive(m_moveInfo);
    return countinueToRun;
}
void ChassisBackwardTest::CompareAndReport()
{
}
