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

class SwerveContainer
{
public:
    SwerveContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    std::unique_ptr<subsystems::CommandSwerveDrivetrain> m_chassis;

    units::meters_per_second_t m_maxSpeed = ChassisConfigMgr::GetInstance()->GetMaxSpeed(); // kSpeedAt12Volts desired top speed
    units::radians_per_second_t m_maxAngularRate = 0.75_tps;                                // 3/4 of a rotation per second max angular velocity

    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    Telemetry logger;

    frc2::CommandPtr m_fieldDrive;
    frc2::CommandPtr m_robotDrive;

    void ConfigureBindings();
};
