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
                                       m_driveToRightReefBranch(nullptr),
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

    // m_driveToNote = dynamic_cast<DriveToNote *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE));
}
void DrivePathPlanner::InitMap()
{
    if (m_chassis != nullptr)
    {
        auto leftReefBranchTuple = make_tuple(dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH)),
                                              ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH,
                                              DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH);
        m_updateOptionToTrajMap[UPDATE_OPTION::LEFT_REEF_BRANCH] = leftReefBranchTuple;

        m_updateOptionToTrajMap[UPDATE_OPTION::RIGHT_REEF_BRANCH] = dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH));
        m_updateOptionToTrajMap[UPDATE_OPTION::CORAL_STATION] = dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_CORAL_STATION));
    }
}
void DrivePathPlanner::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_choreoTrajectoryName = params->GetTrajectoryName();
    m_pathGainsType = params->GetPathGainsType();

    m_updateOption = params->GetUpdateOption();

    m_ntName = string("DrivePathPlanner: ") + m_pathname;
    m_maxTime = params->GetTime();
    m_isVisionDrive = (m_pathname == "RIGHT_REEF_BRANCH");
    m_visionAlignment = params->GetVisionAlignment();
    m_checkForDriveToReef = params->GetUpdateOption() != UPDATE_OPTION::NOTHING;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), m_pathname, m_chassis->GetPose().Rotation().Degrees().to<double>());

    // Start timeout timer for path

    InitMoveInfo();
    m_moveInfo.headingOption = params->GetHeadingOption();

    m_timer.get()->Reset();
    m_timer.get()->Start();
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
        // m_driveToNote = dynamic_cast<DriveToNote *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE));
        trajectory = std::get<TrajectoryDrivePathPlanner *>(m_updateOptionToTrajMap[m_updateOption])->CreateTrajectory();

        m_moveInfo.driveOption = ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH;

        m_driveToRightReefBranch->InitFromTrajectory(m_moveInfo, trajectory);
        m_maxTime += m_moveInfo.pathplannerTrajectory.getTotalTime();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Total time", "Total time", m_maxTime.value());
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
            trajectory = path.get()->generateTrajectory(speed, pose.Rotation(), m_chassis->GetRobotConfig());
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), string("Path not found"), m_pathname);
        }
    }

    auto endstate = trajectory.getEndState();
    m_finalPose = endstate.pose;
    m_moveInfo.pathplannerTrajectory = trajectory;
    m_totalTrajectoryTime = trajectory.getTotalTime();
}
void DrivePathPlanner::Run()
{
    if (m_chassis != nullptr)
    {
        m_chassis->Drive(m_moveInfo);
    }
}

bool DrivePathPlanner::IsDone()
{

    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {
        return true;
    }

    if (m_checkForDriveToReef && !m_isVisionDrive)
    {
        CheckForDriveToReefBranch();
    }

    if (m_isVisionDrive)
    {
        return m_driveToRightReefBranch->IsDone();
    }
    auto *trajectoryDrive = dynamic_cast<TrajectoryDrivePathPlanner *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER));

    return trajectoryDrive != nullptr ? trajectoryDrive->IsDone() : false;
}

// TODO rework for REEF, CORAL_STATION and later PROCESSOR and ALGAE
void DrivePathPlanner::CheckForDriveToReefBranch()
{
    // Need to check if there is a Reef Branch
    DragonTargetFinder *dt = DragonTargetFinder::GetInstance();
    auto reefBranch = dt->GetPose(DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH);

    if (reefBranch.has_value())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "X", get<1>(reefBranch.value()).X().value());
        if (get<0>(reefBranch.value()) != DragonTargetFinderData::NOT_FOUND) // see a Reef Branch
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Reef Branch Found: ", true);
            auto branchPose = get<1>(reefBranch.value());

            // check if we see a Reef Branch is one we want to get

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Consider Reef Branch: ", true);

            auto chassispose = m_chassis->GetPose();
            auto distanceToBranch = chassispose.Translation().Distance(branchPose.Translation());

            auto currentTime = m_timer.get()->Get();
            auto percent = currentTime.value() / m_totalTrajectoryTime.value();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "time:", currentTime.value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Done Percent:", static_cast<double>((currentTime.value()) / m_totalTrajectoryTime.value()));
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Distance: ", distanceToBranch.value());

            if (distanceToBranch <= m_distanceThreshold) // switch to drive to Reef Branch
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Branch", "Switch to Drive To Reef Branch: ", true);

                m_pathname = "RIGHT_REEF_BRANCH";
                m_isVisionDrive = true;
                m_visionAlignment = PrimitiveParams::VISION_ALIGNMENT::REEF;
                InitMoveInfo();
            }
            else
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Switch to Drive To Reef Branch: ", false);
            }
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Reef Branch", "Reef Branch Found: ", false);
        }
    }
}
