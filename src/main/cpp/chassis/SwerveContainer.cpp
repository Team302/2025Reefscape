// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "chassis/SwerveContainer.h"
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include "chassis/states/FieldDrive.h"
#include "chassis/states/RobotDrive.h"

SwerveContainer::SwerveContainer() : m_chassis(ChassisConfigMgr::GetInstance()->CreateDrivetrain()),
                                     m_maxSpeed(ChassisConfigMgr::GetInstance()->GetMaxSpeed()),
                                     m_fieldDrive(std::make_unique<FieldDrive>(m_chassis.get(), TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
                                     m_robotDrive(std::make_unique<RobotDrive>(m_chassis.get(), TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate))

{
    if (m_chassis != nullptr)
    {
        ConfigureBindings();
    }
}

void SwerveContainer::ConfigureBindings()
{
    auto controller = TeleopControl::GetInstance();

    m_chassis->SetDefaultCommand(std::move(m_fieldDrive));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_chassis->ApplyRequest([]
                                { return swerve::requests::Idle{}; })
            .IgnoringDisable(true));

    controller->GetCommandTrigger(TeleopControlFunctions::HOLD_POSITION).WhileTrue(m_chassis->ApplyRequest([this]() -> auto &&
                                                                                                           { return brake; }));

    // Point the wheels to a certain direciton, but don't move the chassis.  TO DO, probably update this to not be based on teleop control, but a command we can send (Maybe for starting auton)
    //  controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_RIGHT).WhileTrue(m_chassis->ApplyRequest([this, controller]() -> auto && { return point.WithModuleDirection(frc::Rotation2d{controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD), controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE)}); }));

    // Run SysId routines when holding Select and A,X,Y,B.
    // Note that each routine should be run exactly once in a single log.
    (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_HUMAN_PLAYER_STATION)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kForward)); // A
    (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_LEFT)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kReverse));                 // B
    (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_CLIMB)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kForward));                  // Y
    (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_RIGHT)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kReverse));            // X

    // reset the field-centric heading on left bumper press
    controller->GetCommandTrigger(TeleopControlFunctions::RESET_POSITION).OnTrue(m_chassis->RunOnce([this, controller]
                                                                                                    { m_chassis->SeedFieldCentric(); }));

    controller->GetCommandTrigger(TeleopControlFunctions::ROBOT_ORIENTED_DRIVE).WhileTrue(std::move(m_robotDrive));

    m_chassis->RegisterTelemetry([this](auto const &state)
                                 { logger.Telemeterize(state); });
}

frc2::CommandPtr SwerveContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}
