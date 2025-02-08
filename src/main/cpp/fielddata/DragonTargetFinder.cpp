
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

#include <optional>
#include <string>

#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/ISwerveDriveOrientation.h"
#include "fielddata/CoralStationHelper.h"
#include "fielddata/DragonTargetFinder.h"
#include "fielddata/FieldConstants.h"
#include "fielddata/ReefHelper.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "utils/FMSData.h"

using frc::Pose2d;
using frc::Pose3d;
using std::make_tuple;
using std::optional;
using std::string;
using std::tuple;

DragonTargetFinder *DragonTargetFinder::m_instance = nullptr;
DragonTargetFinder *DragonTargetFinder::GetInstance()
{
    if (DragonTargetFinder::m_instance == nullptr)
    {
        DragonTargetFinder::m_instance = new DragonTargetFinder();
    }
    return DragonTargetFinder::m_instance;
}

DragonTargetFinder::DragonTargetFinder() : m_chassis(ChassisConfigMgr::GetInstance()->GetCurrentChassis()), m_vision(DragonVision::GetDragonVision())
{
}

optional<tuple<DragonTargetFinderData, Pose2d>> DragonTargetFinder::GetPose(DragonTargetFinderTarget item)
{
    tuple<DragonTargetFinderData, Pose2d> targetInfo;

    auto fieldconst = FieldConstants::GetInstance();

    if (item == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH ||
        item == DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH ||
        item == DragonTargetFinderTarget::CLOSEST_REEF_ALGAE)
    {
        // call reef helper to find the appropriate closest side of the reef,
        // its corresponding APRILTAG ID and the field constant identifier
        auto taginfo = ReefHelper::GetInstance()->GetNearestReefTag();
        if (taginfo.has_value())
        {
            auto tag = taginfo.value();
            auto tagpose{fieldconst->GetAprilTagPose(tag).ToPose2d()};

            if (item == DragonTargetFinderTarget::CLOSEST_REEF_ALGAE)
            {
                return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, tagpose);
            }
            else if (item == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH)
            {
                auto leftbranch = ReefHelper::GetInstance()->GetNearestLeftReefBranch(tag);
                if (leftbranch.has_value())
                {
                    auto leftbranchpose = fieldconst->GetFieldElementPose(leftbranch.value()).ToPose2d();
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, leftbranchpose);
                }
            }
            else // right branch
            {
                auto rightbranch = ReefHelper::GetInstance()->GetNearestRightReefBranch(tag);
                if (rightbranch.has_value())
                {
                    auto rightbranchpose = fieldconst->GetFieldElementPose(rightbranch.value()).ToPose2d();
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, rightbranchpose);
                }
            }
        }
        auto pose2d = Pose2d();
        targetInfo = make_tuple(DragonTargetFinderData::NOT_FOUND, pose2d);
        return targetInfo;
    }
    else if (item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE ||
             item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE ||
             item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE)
    {
        // call coral station helper to find the appropriate the coral station,
        // its corresponding APRILTAG ID and the field constant identifier
        auto taginfo = CoralStationHelper::GetInstance()->GetNearestCoralStationTag();
        if (taginfo.has_value())
        {
            auto tag = taginfo.value();
            auto tagpose{fieldconst->GetAprilTagPose(tag).ToPose2d()};
            if (item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE)
            {
                return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, tagpose);
            }
            else if (item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE)
            {
                auto sidewall = CoralStationHelper::GetInstance()->GetNearestSideWallCoralStation(tag);
                if (sidewall.has_value())
                {
                    auto sidewallpose = fieldconst->GetFieldElementPose(sidewall.value()).ToPose2d();
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, sidewallpose);
                }
            }
            else // CLOSEST_CORAL_STATION_ALLIANCE_SIDE
            {
                auto alliance = CoralStationHelper::GetInstance()->GetNearestAllianceWallCoralStation(tag);
                if (alliance.has_value())
                {
                    auto alliancepose = fieldconst->GetFieldElementPose(alliance.value()).ToPose2d();
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, alliancepose);
                }
            }
        }
    }
    else if (item == DragonTargetFinderTarget::LEFT_CAGE ||
             item == DragonTargetFinderTarget::CENTER_CAGE ||
             item == DragonTargetFinderTarget::RIGHT_CAGE)
    {
        // call cage helper to find the appropriate the cage,
        // its corresponding APRILTAG ID and the field constant identifier
    }

    auto pose2d = Pose2d();
    targetInfo = make_tuple(DragonTargetFinderData::NOT_FOUND, pose2d);
    return targetInfo;
}

void DragonTargetFinder::SetCorrection(ChassisMovement &chassisMovement,
                                       SwerveChassis *chassis,
                                       units::angle::degree_t target,
                                       double kp)
{
    chassis->SetStoredHeading(target);
    if (chassis != nullptr)
    {
        units::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;
        if (std::abs(rot.to<double>()) < 0.1)
        {
            chassisMovement.chassisSpeeds.omega = units::radians_per_second_t(0.0);

            auto correction = ISwerveDriveOrientation::CalcHeadingCorrection(chassis->GetStoredHeading(), kp);
            chassisMovement.chassisSpeeds.omega += correction;
        }
    }
}

int DragonTargetFinder::GetAprilTag(DragonVision::VISION_ELEMENT item)
{
    /** TODO JW come back to
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        auto itr = blueMap.find(item);
        if (itr != blueMap.end())
        {
            return itr->second;
        }
    }
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kRed)
    {
        auto itr = redMap.find(item);
        if (itr != redMap.end())
        {
            return itr->second;
        }
    }
    **/
    return -1;
}

frc::Pose2d DragonTargetFinder::GetAprilTagPose(DragonVision::VISION_ELEMENT item)
{
    auto aprilTag = GetAprilTag(item);
    if (aprilTag > 0)
    {
        auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
        if (pose)
        {
            return pose.value().ToPose2d();
        }
    }
    return {};
}

units::angle::degree_t DragonTargetFinder::AdjustRobotRelativeAngleForIntake(units::angle::degree_t angle)
{
    auto robotRelativeAngle = angle;
    if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
    {
        robotRelativeAngle += units::angle::degree_t(180.0);
    }
    else if (robotRelativeAngle >= units::angle::degree_t(90.0))
    {
        robotRelativeAngle -= units::angle::degree_t(180.0);
    }
    return robotRelativeAngle;
}
