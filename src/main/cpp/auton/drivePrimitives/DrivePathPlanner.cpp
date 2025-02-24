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
#include "auton/drivePrimitives/DrivePathPlanner.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "fielddata/DragonTargetFinder.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "chassis/states/DriveToCoralStation.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "vision/DragonVision.h"
#include "state/StateMgr.h"
// #include "mechanisms/MechanismTypes.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/states/RobotDrive.h"

// third party includes
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"
#include "pathplanner/lib/config/ModuleConfig.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/config/RobotConfig.h"

using pathplanner::ModuleConfig;
using pathplanner::PathPlannerPath;
using pathplanner::PathPlannerTrajectory;

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePathPlanner::DrivePathPlanner() : IPrimitive(),
                                       m_chassis(nullptr),
                                       m_timer(make_unique<Timer>()),
                                       m_trajectory(),
                                       m_pathname(),
                                       m_choreoTrajectoryName(),
                                       m_pathGainsType(ChassisOptionEnums::PathGainsType::LONG),
                                       // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                                       m_maxTime(units::time::second_t(-1.0)),
                                       m_ntName("DrivePathPlanner"),
                                       m_isVisionDrive(false),
                                       m_visionAlignment(PrimitiveParams::VISION_ALIGNMENT::UNKNOWN)

{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
TrajectoryDrivePathPlanner *DrivePathPlanner::GetDriveToObject(ChassisOptionEnums::DriveStateType driveToType)
{
    switch (driveToType)
    {
    case ChassisOptionEnums::DRIVE_TO_CORAL_STATION:
        return dynamic_cast<DriveToCoralStation *>(m_chassis->GetSpecifiedDriveState(driveToType));
    case ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH:
        return dynamic_cast<DriveToRightReefBranch *>(m_chassis->GetSpecifiedDriveState(driveToType));
    case ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH:
        return dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(driveToType));
    default:
        return nullptr;
    }
}
int DrivePathPlanner::FindDriveToZoneIndex(ZoneParamsVector zones)
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
void DrivePathPlanner::Init(PrimitiveParams *params)
{
    m_zone = nullptr;
    m_updateTimeLatch = false;
    m_driveToObject = nullptr;

    auto index = FindDriveToZoneIndex(params->GetZones());
    if (index != -1)
    {
        m_zone = params->GetZones()[index];
    }

    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_choreoTrajectoryName = params->GetTrajectoryName();
    m_pathGainsType = params->GetPathGainsType();

    m_ntName = string("DrivePathPlanner: ") + m_pathname;
    m_maxTime = params->GetTime();

    m_isVisionDrive = false;
    m_visionAlignment = params->GetVisionAlignment();

    if (m_zone != nullptr)
    {
        m_driveToObject = GetDriveToObject(m_zone->GetPathUpdateOption());
        m_checkForDriveToUpdate = true;
    }
    else
    {
        m_driveToObject = nullptr;
        m_checkForDriveToUpdate = false;
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), m_pathname, m_chassis->GetPose().Rotation().Degrees().to<double>());

    // Start timeout timer for path

    InitMoveInfo();
    m_moveInfo.headingOption = params->GetHeadingOption();

    m_timer.get()->Reset();
    m_timer.get()->Start();

    LogMoveInfo();
}

void DrivePathPlanner::LogMoveInfo()
{
    currentPrim++;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner " + to_string(currentPrim), "Drive Option", m_moveInfo.driveOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner " + to_string(currentPrim), "Gain Type", m_moveInfo.pathnamegains);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner " + to_string(currentPrim), "Heading Option", m_moveInfo.headingOption);
}
void DrivePathPlanner::DataLog(uint64_t timestamp)
{
    LogStringData(timestamp, DragonDataLoggerSignals::StringSignals::AUTON_PATH_NAME, m_pathname);
}

void DrivePathPlanner::InitMoveInfo()
{
    m_moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER;

    m_moveInfo.pathnamegains = m_pathGainsType;

    auto pose = m_chassis->GetPose();
    auto speed = m_chassis->GetChassisSpeeds();

    pathplanner::PathPlannerTrajectory trajectory;

    if (m_isVisionDrive)
    {
        m_moveInfo.driveOption = m_zone->GetPathUpdateOption();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "Drive Option", m_zone->GetPathUpdateOption());
    }
    else
    {
        shared_ptr<PathPlannerPath> path;
        if (m_pathname.empty())
            path = AutonUtils::GetPathFromTrajectory(m_choreoTrajectoryName);
        else
            path = AutonUtils::GetPathFromPathFile(m_pathname);

        if (AutonUtils::IsValidPath(path))
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DrivePathPlanner"), string("Valid Path"), true);

            trajectory = path.get()->generateTrajectory(speed, pose.Rotation(), m_chassis->GetRobotConfig());
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), string("Path not found"), m_pathname);
        }
        m_moveInfo.pathplannerTrajectory = trajectory;
        auto endstate = trajectory.getEndState();
        m_finalPose = endstate.pose;
        m_totalTrajectoryTime = trajectory.getTotalTime();
    }
}
void DrivePathPlanner::Run()
{
    if (m_chassis != nullptr)
    {
        m_chassis->Drive(m_moveInfo);
    }

    if (m_isVisionDrive && !m_moveInfo.pathplannerTrajectory.getStates().empty() && !m_updateTimeLatch)
    {
        m_maxTime += m_moveInfo.pathplannerTrajectory.getTotalTime();
        m_updateTimeLatch = true;
    }
}

bool DrivePathPlanner::IsDone()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "Max Time", m_maxTime.value());
    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {

        return true;
    }

    if (m_checkForDriveToUpdate && !m_isVisionDrive)
    {
        CheckForDriveTo();
    }

    // auto *trajectoryDrive = dynamic_cast<TrajectoryDrivePathPlanner *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER));

    // return trajectoryDrive != nullptr ? trajectoryDrive->IsDone() : false;
    return false;
}

void DrivePathPlanner::CheckForDriveTo()
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
bool DrivePathPlanner::IsInZone()
{
    if (m_zone->GetZoneMode() != AutonGrid::ZoneMode::NOTHING && m_chassis != nullptr)
    {

        if (m_zone->GetZoneMode() == AutonGrid::ZoneMode::CIRCLE)
        {
            return AutonGrid::GetInstance()->IsPoseInZone(m_zone->GetCircleZonePose(),
                                                          m_zone->GetRadius(),
                                                          m_chassis->GetPose());
        }
        else if (m_zone->GetZoneMode() == AutonGrid::ZoneMode::RECTANGLE)
        {
            return AutonGrid::GetInstance()->IsPoseInZone(m_zone->GetXGrid1(),
                                                          m_zone->GetXGrid2(),
                                                          m_zone->GetYGrid1(),
                                                          m_zone->GetYGrid2(),
                                                          m_chassis->GetPose());
        }
    }
    return false;
}