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

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "chassis/generated/Telemetry.h"
#include "chassis/ChassisConfigMgr.h"
#include "teleopcontrol/TeleopControl.h"

class SwerveContainer : IRobotStateChangeSubscriber
{
public:
    SwerveContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;

    units::meters_per_second_t m_maxSpeed;
    units::radians_per_second_t m_maxAngularRate = 1_tps;

    swerve::requests::SwerveDriveBrake m_brakeRequest{};

    Telemetry logger;

    frc2::CommandPtr m_fieldDrive;
    frc2::CommandPtr m_robotDrive;
    frc2::CommandPtr m_polarDrive;
    frc2::CommandPtr m_driveToCoralStation;
    frc2::CommandPtr m_driveToCoralLeftBranch;
    frc2::CommandPtr m_driveToCoralRightBranch;
    frc2::CommandPtr m_driveToBarge;
    frc2::CommandPtr m_driveToLeftCage;
    frc2::CommandPtr m_driveToRightCage;
    frc2::CommandPtr m_driveToCenterCage;

    bool m_climbMode = false;

    void ConfigureBindings();
    void SetSysIDBinding(TeleopControl *controller);
    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;
};
