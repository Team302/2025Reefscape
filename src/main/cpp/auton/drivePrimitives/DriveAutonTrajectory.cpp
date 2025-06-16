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

// C++
#include <string>

// FRC Includes
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/TrajectoryUtil.h"
#include "units/angular_velocity.h"
#include "wpi/fs.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"

// 302 Includes
#include "auton/drivePrimitives/AutonUtils.h"
#include "auton/drivePrimitives/DriveAutonTrajectory.h"

#include "chassis/ChassisConfigMgr.h"
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "fielddata/DragonTargetFinder.h"
#include "chassis/states/TrajectoryDrive.h"
#include "chassis/states/DriveToTarget.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "vision/DragonVision.h"
#include "state/StateMgr.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/states/RobotDrive.h"

// third party includes

using namespace std;
using namespace frc;

using namespace wpi::math;

DriveAutonTrajectory::DriveAutonTrajectory() : IPrimitive(),
                                               m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                               m_timer(make_unique<Timer>()),
                                               m_choreoTrajectoryName(),
                                               // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                                               m_maxTime(units::time::second_t(-1.0)),
                                               m_ntName("DriveAutonTrajectory"),
                                               m_isVisionDrive(false),
                                               m_visionAlignment(PrimitiveParams::VISION_ALIGNMENT::UNKNOWN)

{
}
DriveToTarget *DriveAutonTrajectory::GetDriveToObject(ChassisOptionEnums::DriveStateType driveToType)
{
    switch (driveToType)
    {
    case ChassisOptionEnums::DriveStateType::DRIVE_TO_CORAL_STATION:
        return dynamic_cast<DriveToCoralStation *>(m_chassis->GetSpecifiedDriveState(driveToType));
    case ChassisOptionEnums::DriveStateType::DRIVE_TO_RIGHT_REEF_BRANCH:
        return dynamic_cast<DriveToRightReefBranch *>(m_chassis->GetSpecifiedDriveState(driveToType));
    case ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_REEF_BRANCH:
        return dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(driveToType));
    case ChassisOptionEnums::DriveStateType::DRIVE_TO_BARGE:
        return dynamic_cast<DriveToBarge *>(m_chassis->GetSpecifiedDriveState(driveToType));
    default:
        return nullptr;
    }
}
int DriveAutonTrajectory::FindDriveToZoneIndex(ZoneParamsVector zones)
{
    if (!zones.empty())
    {
        for (unsigned int i = 0; i < zones.size(); i++)
        {
            if (zones[i]->GetPathUpdateOption() != ChassisOptionEnums::STOP_DRIVE)
            {
                return i;
            }
        }
    }
    // if we don't have an update option
    return -1;
}
void DriveAutonTrajectory::Init(PrimitiveParams *params)
{
    m_zone = nullptr;
    m_updateTimeLatch = false;
    m_driveToObject = nullptr;
    m_checkForDriveToUpdate = false;

    auto index = FindDriveToZoneIndex(params->GetZones());
    if (index != -1)
    {
        m_zone = params->GetZones()[index];
        if (m_zone != nullptr)
        {
            m_driveToObject = GetDriveToObject(m_zone->GetPathUpdateOption());
            m_checkForDriveToUpdate = true;
        }
    }

    m_choreoTrajectoryName = params->GetTrajectoryName();

    m_ntName = string("DriveAutonTrajectory: ") + m_choreoTrajectoryName;
    m_maxTime = params->GetTime();

    m_isVisionDrive = false;
    m_visionAlignment = params->GetVisionAlignment();

    // Start timeout timer for path

    InitMoveInfo();
    m_moveInfo.headingOption = params->GetHeadingOption();

    m_timer.get()->Reset();
    m_timer.get()->Start();
}

void DriveAutonTrajectory::LogMoveInfo()
{
    m_currentPrim++;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveAutonTrajectory " + to_string(m_currentPrim), "Drive Option", m_moveInfo.driveOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveAutonTrajectory " + to_string(m_currentPrim), "Heading Option", m_moveInfo.headingOption);
}
void DriveAutonTrajectory::DataLog(uint64_t timestamp)
{
    LogStringData(timestamp, DragonDataLogger::StringSignals::AUTON_PATH_NAME, m_choreoTrajectoryName);
}

void DriveAutonTrajectory::InitMoveInfo()
{
    if (m_isVisionDrive)
    {
        m_moveInfo.driveOption = m_zone->GetPathUpdateOption();

        if (m_driveToObject != nullptr)
        {
            m_driveToObject->Init(m_moveInfo);
        }
    }
    else
    {
        m_moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
        m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER;

        auto trajectory = AutonUtils::GetTrajectoryFromPathFile(m_choreoTrajectoryName);

        if (trajectory.has_value())
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DriveAutonTrajectory"), string("Valid Path"), true);

            m_moveInfo.trajectory = trajectory;
            m_finalPose = trajectory.value().GetFinalPose().value();
            m_totalTrajectoryTime = trajectory.value().GetTotalTime();
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DriveAutonTrajectory"), string("Path not found"), m_choreoTrajectoryName);
        }
    }
}
void DriveAutonTrajectory::Run()
{
    if (m_chassis != nullptr)
    {
        if (m_checkForDriveToUpdate && !m_isVisionDrive)
        {
            CheckForDriveTo();
        }

        // m_chassis->Drive(m_moveInfo);
    }
}

bool DriveAutonTrajectory::IsDone()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveAutonTrajectory", "Max Time", m_maxTime.value());
    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {

        return true;
    }

    if (m_isVisionDrive && m_driveToObject != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveAutonTrajectory", m_driveToObject->GetDriveStateName(), m_driveToObject->IsDone());
        return m_driveToObject->IsDone();
    }
    else if (!m_isVisionDrive)
    {
        auto trajDrive = dynamic_cast<TrajectoryDrive *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::TRAJECTORY_DRIVE_PLANNER));

        if (((m_timer.get()->Get() / m_maxTime) < 0.90))
        {
            return false;
        }
        else if (trajDrive != nullptr)
        {
            return trajDrive->IsDone();
        }
    }

    return true; // TODO: Add logic for IsDone() from TrajectoryDrive
}

void DriveAutonTrajectory::CheckForDriveTo()
{
    if (IsInZone()) // switch to the selected drive to option
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Branch", "Switch to Drive To Reef Branch: ", true);

        m_isVisionDrive = true;

        // will come back to later
        m_visionAlignment = PrimitiveParams::VISION_ALIGNMENT::REEF;
        InitMoveInfo();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Switch to Drive To Reef Branch: ", false);
    }
}
bool DriveAutonTrajectory::IsInZone()
{
    if (m_zone->GetZoneMode() != ZoneMode::NOTHING && m_chassis != nullptr)
    {

        return m_zone->IsPoseInZone(m_chassis->GetPose());
    }
    return false;
}