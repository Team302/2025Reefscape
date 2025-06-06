// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "chassis/CommandSwerveDrivetrain.h"
#include "chassis/pose/Telemetry.h"
#include "chassis/ChassisConfigMgr.h"
#include "teleopcontrol/TeleopControl.h"

class RobotContainer
{
private:
    units::meters_per_second_t m_maxSpeed = ChassisConfigMgr::GetInstance()->GetMaxSpeed(); // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps;                                  // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                               .WithDeadband(m_maxSpeed * 0.1)
                                               .WithRotationalDeadband(MaxAngularRate * 0.1)                     // Add a 10% deadband
                                               .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the chassis, otherwise we need to
     *       define a destructor to un-register the telemetry from the chassis */
    Telemetry logger;

    frc2::CommandXboxController *joystick = TeleopControl::GetInstance()->GetHybridController()->GetCommandController();

public:
    std::unique_ptr<subsystems::CommandSwerveDrivetrain> m_chassis;

    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();
};
