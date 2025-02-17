// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <string>

#include <Robot.h>

#include <frc/RobotController.h>

#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/HolonomicDrive.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/SwerveChassis.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "feedback/DriverFeedback.h"
#include "RobotIdentifier.h"
#include "state/RobotState.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "utils/DragonPower.h"
#include "utils/DragonPower.h"
#include "utils/logging/debug/LoggableItemMgr.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/logging/trace/DataTrace.h"
#include "utils/PeriodicLooper.h"
#include "utils/sensors/SensorData.h"
#include "utils/sensors/SensorData.h"
#include "utils/sensors/SensorDataMgr.h"
#include "utils/sensors/SensorDataMgr.h"
#include "vision/definitions/CameraConfig.h"
#include "vision/definitions/CameraConfigMgr.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"

using std::string;

void Robot::RobotInit()
{
    isFMSAttached = frc::DriverStation::IsFMSAttached();

    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("arrived"));
        InitializeDataTracing();
    }

    m_controller = nullptr;

    InitializeRobot();
    InitializeAutonOptions();
    InitializeDriveteamFeedback();

    m_datalogger = DragonDataLoggerMgr::GetInstance();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("end"));
    }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    SensorDataMgr::GetInstance()->CacheData();

    isFMSAttached = isFMSAttached ? true : frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        LoggableItemMgr::GetInstance()->LogData();
        Logger::GetLogger()->PeriodicLog();
    }

    if (m_datalogger != nullptr && !frc::DriverStation::IsDisabled())
    {
        m_datalogger->PeriodicDataLog();
    }

    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

    UpdateDriveTeamFeedback();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("arrived"));
    }

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("end"));
    }
}

void Robot::AutonomousPeriodic()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousPeriodic"), string("arrived"));
    }

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousPeriodic"), string("end"));
    }
}

void Robot::TeleopInit()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("arrived"));
    }

    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Init();
    }

    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("end"));
    }
}

void Robot::TeleopPeriodic()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("arrived"));
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Run();
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("end"));
    }
}

void Robot::DisabledInit()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("arrived"));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("end"));
    }
}

void Robot::DisabledPeriodic()
{
    m_dragonswerveposeestimator->CalculateInitialPose();
}

void Robot::TestInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TestInit"), string("arrived"));
}

void Robot::TestPeriodic()
{
}

void Robot::SimulationInit()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::SimulationPeriodic()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::InitializeRobot()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);
    ChassisConfigMgr::GetInstance()->InitChassis(static_cast<RobotIdentifier>(teamNumber));
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
    m_holonomic = nullptr;
    if (m_chassis != nullptr)
    {
        m_holonomic = new HolonomicDrive();
    }
    m_dragonPower = DragonPower::GetInstance();

    // initialize cameras
    CameraConfigMgr::GetInstance()->InitCameras(static_cast<RobotIdentifier>(teamNumber));
    // auto vision = DragonVision::GetDragonVision();

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();
}

void Robot::InitializeAutonOptions()
{
    m_cyclePrims = new CyclePrimitives(); // intialize auton selections
    m_previewer = new AutonPreviewer(m_cyclePrims);
}
void Robot::InitializeDriveteamFeedback()
{
    m_field = DragonField::GetInstance(); // TODO: move to drive team feedback
}

void Robot::UpdateDriveTeamFeedback()
{
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }
    if (m_field != nullptr && m_chassis != nullptr)
    {
        m_field->UpdateRobotPosition(m_chassis->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
    }
    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }
}

void Robot::InitializeDataTracing()
{
    DataTrace::GetInstance()->Connect();
}
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
