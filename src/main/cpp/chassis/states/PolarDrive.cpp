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
#include "chassis/states/PolarDrive.h"
#include "frc/geometry/Pose2d.h"
#include "chassis/ChassisConfigMgr.h"
PolarDrive::PolarDrive(subsystems::CommandSwerveDrivetrain *chassis,
                       TeleopControl *controller,
                       units::velocity::meters_per_second_t maxSpeed) : m_chassis(chassis),
                                                                        m_controller(controller),
                                                                        m_maxSpeed(maxSpeed)

{
    AddRequirements(m_chassis);
}

void PolarDrive::Initialize()
{
    m_reefCenter = frc::Translation2d{13.10_m, 4.10_m}; // fixed for now, this can be set once DragonTargetFinder is in place

    double heading_kP = 7.5;
    double heading_kI = 2.0;
    double heading_kD = 0.0;
    m_polarDrive.WithHeadingPID(heading_kP, heading_kI, heading_kD);
}

void PolarDrive::Execute()
{
    frc::Pose2d currentPose = m_chassis->GetPose();

    units::meter_t x_diff = m_reefCenter.X() - currentPose.X();
    units::meter_t y_diff = m_reefCenter.Y() - currentPose.Y();
    frc::Rotation2d angleToTarget = units::math::atan2(y_diff, x_diff);

    auto radialVelocity = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD) * m_maxSpeed;
    auto angularVelocity = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE) * m_maxSpeed;
    // 4. Send the control request
    m_chassis->SetControl(
        m_polarDrive
            .WithVelocityX(radialVelocity)
            .WithVelocityY(angularVelocity)
            .WithTargetDirection(angleToTarget));
}

bool PolarDrive::IsFinished()
{
    return false; // Run until interrupted
}

void PolarDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}