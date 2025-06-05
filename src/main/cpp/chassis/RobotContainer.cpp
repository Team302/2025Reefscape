// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "chassis/RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    m_chassis = ChassisConfigMgr::GetInstance()->CreateDrivetrain();
    m_maxSpeed = ChassisConfigMgr::GetInstance()->GetMaxSpeed();
    if (m_chassis != nullptr)
    {
        ConfigureBindings();
    }
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_chassis->SetDefaultCommand(
        // Drivetrain will execute this command periodically
        m_chassis->ApplyRequest([this]() -> auto &&
                                {
                                    return drive.WithVelocityX(-joystick.GetLeftY() * m_maxSpeed)    // Drive forward with negative Y (forward)
                                        .WithVelocityY(-joystick.GetLeftX() * m_maxSpeed)            // Drive left with negative X (left)
                                        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                                }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_chassis->ApplyRequest([]
                                { return swerve::requests::Idle{}; })
            .IgnoringDisable(true));

    joystick.A().WhileTrue(m_chassis->ApplyRequest([this]() -> auto &&
                                                   { return brake; }));
    joystick.B().WhileTrue(m_chassis->ApplyRequest([this]() -> auto &&
                                                   { return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()}); }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(m_chassis->RunOnce([this]
                                                    { m_chassis->SeedFieldCentric(); }));

    m_chassis->RegisterTelemetry([this](auto const &state)
                                 { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}
