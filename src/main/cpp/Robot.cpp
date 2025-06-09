// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "utils/logging/debug/Logger.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "frc/DriverStation.h"
#include "utils/RoboRio.h"
#include "state/RobotState.h"

Robot::Robot()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();

    m_controller = nullptr;

    InitializeDriveteamFeedback();

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
    // int32_t teamNumber = frc::RobotController::GetTeamNumber();
    // FieldConstants::GetInstance();
    RoboRio::GetInstance();

    // m_dragonswerveposeestimator = nullptr;

    // MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);

    // initialize cameras
    // CameraConfigMgr::GetInstance()->InitCameras(static_cast<RobotIdentifier>(teamNumber));
    // auto vision = DragonVision::GetDragonVision();

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();
}

void Robot::InitializeAutonOptions()
{
    // m_cyclePrims = new CyclePrimitives(); // intialize auton selections
    // m_previewer = new AutonPreviewer(m_cyclePrims);
}
void Robot::InitializeDriveteamFeedback()
{
    m_field = DragonField::GetInstance(); // TODO: move to drive team feedback
}

void Robot::UpdateDriveTeamFeedback()
{
    // if (m_previewer != nullptr)
    // {
    //     m_previewer->CheckCurrentAuton();
    // }
    // if (m_field != nullptr && m_dragonswerveposeestimator != nullptr)
    // {
    //     m_field->UpdateRobotPosition(m_dragonswerveposeestimator->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
    // }
    // auto feedback = DriverFeedback::GetInstance();
    // if (feedback != nullptr)
    // {
    //     feedback->UpdateFeedback();
    // }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
