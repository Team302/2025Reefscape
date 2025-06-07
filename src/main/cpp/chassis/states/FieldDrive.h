#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "chassis/CommandSwerveDrivetrain.h"
#include "teleopcontrol/TeleopControl.h"
#include <units/velocity.h>
#include <units/angular_velocity.h>

class FieldDrive : public frc2::CommandHelper<frc2::Command, FieldDrive>
{
public:
    // Constructor now takes a raw pointer to the subsystem
    FieldDrive(subsystems::CommandSwerveDrivetrain *chassis,
               TeleopControl *controller,
               units::velocity::meters_per_second_t maxSpeed,
               units::angular_velocity::degrees_per_second_t maxAngularRate);

    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    // Member is a simple raw pointer
    subsystems::CommandSwerveDrivetrain *m_chassis;
    TeleopControl *m_controller;
    units::velocity::meters_per_second_t m_maxSpeed;
    units::angular_velocity::degrees_per_second_t m_maxAngularRate;

    // The request object itself
    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                   // Add a 10% deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
};