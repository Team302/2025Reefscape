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
//=====================================================================================================================================================

// C++ Includes
#include <string>
#include <tuple>

// FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "vision/DragonVision.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/DriveToNote.h"
#include "utils/FMSData.h"
#include "vision/DragonVisionStructs.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/DragonTargetFinder.h" // #include "mechanisms/noteManager/decoratormods/noteManager.h"
#include "chassis/SwerveChassis.h"

#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"

DriveToNote::DriveToNote(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner)
    : TrajectoryDrivePathPlanner(robotDrive)
{
}

void DriveToNote::Init(ChassisMovement &chassisMovement)
{

    m_trajectory = CreateDriveToNote();
    InitFromTrajectory(chassisMovement, m_trajectory);
}

std::string DriveToNote::GetDriveStateName() const
{
    return std::string("DriveToNote");
}

void DriveToNote::InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory)
{
    m_trajectory = trajectory;
    if (!m_trajectory.getStates().empty())
    {
        ResetIntakeNoteTimer();
        chassisMovement.pathplannerTrajectory = m_trajectory;
        chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::LONG;
        TrajectoryDrivePathPlanner::Init(chassisMovement);
    }
    else if (frc::DriverStation::IsTeleopEnabled())
    {
        chassisMovement.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
        chassisMovement.headingOption = ChassisOptionEnums::MAINTAIN;
    }
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote()
{
    pathplanner::PathPlannerTrajectory trajectory;
    /**
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        auto info = DragonTargetFinder::GetInstance()->GetPose(DragonVision::VISION_ELEMENT::NOTE);
        auto type = get<0>(info);
        auto data = get<1>(info);
        if (type == DragonTargetFinder::TARGET_INFO::VISION_BASED)
        {
            trajectory = CreateDriveToNoteTrajectory(m_chassis->GetPose(), data);
        }
    }
    return trajectory;
    **/
    return trajectory;
}
pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNoteTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose)
{
    pathplanner::PathPlannerTrajectory trajectory;
    /**
    std::vector<frc::Pose2d> poses{currentPose2d, targetPose};

    DragonVisionStructLogger::logPose2d("current pose", currentPose2d);
    DragonVisionStructLogger::logPose2d("note pose", targetPose);

    std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
    auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                      PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                      GoalEndState(0.0_mps, targetPose.Rotation().Degrees(), true));
    notepath->preventFlipping = true;

    return notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
`   **/
    return trajectory;
}

bool DriveToNote::IsDone()
{
    /**
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto noteStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER);
        if (noteStateMgr != nullptr)
        {
            if (TrajectoryDrivePathPlanner::IsDone())
            {
                IntakeNoteTimerIncrement();

                return dynamic_cast<noteManager *>(noteStateMgr)->HasNote() || m_intakeNoteTimer >= m_finishTime;
            }
            else
            {
                return dynamic_cast<noteManager *>(noteStateMgr)->HasNote();
            }
        }
    }
    **/
    return false;
}
