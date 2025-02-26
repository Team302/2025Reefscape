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

#pragma once

// C++
#include <optional>
#include <tuple>
#include <vector>

// FRC Includes
#include "frc/geometry/Pose2d.h"
#include "units/angle.h"

// Team302 Includes
#include "chassis/states/RobotDrive.h"
#include "chassis/ChassisOptionEnums.h"
#include "fielddata/DragonTargetFinder.h"
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"

class DriveToFieldElement : public TrajectoryDrivePathPlanner
{
public:
    DriveToFieldElement(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner);

    pathplanner::PathPlannerTrajectory
    CreateTrajectory(std::optional<std::tuple<DragonTargetFinderData, frc::Pose2d>> info);

    void Init(ChassisMovement &chassisMovement) override;
    void InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory) override;
    pathplanner::PathPlannerTrajectory GetTrajectory() const { return m_trajectory; }
    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(ChassisMovement &chassisMovement) override;

protected:
    virtual DragonTargetFinderTarget GetDriveToTarget() const = 0;
    virtual ChassisOptionEnums::DriveStateType GetDriveStateType() const = 0;
    virtual ChassisOptionEnums::HeadingOption GetHeadingOption() const = 0;
    virtual units::angle::degree_t GetModifiedHeadingValue(units::angle::degree_t calculatedHeading) { return (calculatedHeading - 180_deg); }

private:
    void InitChassisMovement(ChassisMovement &chassisMovement);

    pathplanner::PathPlannerTrajectory CreateDriveToFieldElementTrajectory(frc::Pose2d currentPose, frc::Pose2d csaPose);

    pathplanner::PathPlannerTrajectory m_trajectory;
    DragonTargetFinderData m_currentType = DragonTargetFinderData::NOT_FOUND;
    std::optional<frc::Pose2d> m_endPose = std::nullopt;
};
