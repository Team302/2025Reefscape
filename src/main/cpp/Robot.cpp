// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "utils/logging/debug/Logger.h"
#include "teleopcontrol/TeleopControl.h"
#include "frc/DriverStation.h"

Robot::Robot()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();

    m_controller = nullptr;

    isFMSAttached = frc::DriverStation::IsFMSAttached();
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();

    isFMSAttached = isFMSAttached ? true : frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->PeriodicLog();
    }

    auto hybridController = TeleopControl::GetInstance()->GetHybridController()->GetCommandController();
    auto driveForward = hybridController->GetLeftY();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::InitializeRobot()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
