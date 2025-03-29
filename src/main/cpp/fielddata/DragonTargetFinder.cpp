
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
#include "fielddata/FieldElementCalculator.h"
#include "fielddata/ReefHelper.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include "utils/FMSData.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/BargeHelper.h"

// Debugging
#include "utils/logging/debug/Logger.h"

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
    SetChassis();
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
            auto tagpose{fieldconst->GetAprilTagPose(tag)};
            auto visTagPose{m_vision->GetAprilTagPose(tag)};
            m_switchToVision = SwitchToVision(visTagPose);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonTargetFinder", "SwitchToVision", m_switchToVision ? "true" : "false");

            if (item == DragonTargetFinderTarget::CLOSEST_REEF_ALGAE)
            {
                if (m_switchToVision)
                {
                    units::angle::degree_t fieldRelativeAngle = m_chassis->GetYaw() - visTagPose.value().ToPose2d().Rotation().Degrees(); // Need to verify if it works for Red and Blue and all the way around the reef
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonTargetFinder", "Field Realitve Angle", fieldRelativeAngle.value());
                    // return make_tuple(DragonTargetFinderData::VISION_BASED, visTagPose.value().ToPose2d());
                }
                return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, tagpose.ToPose2d());
            }
            else if (item == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH)
            {
                // TODO:  Update when we have reef machine learning Add another DragonTargetFinderData Enum
                // Have a vision pose of the tag, calculate the offset to the reef branch
                if (m_switchToVision)
                {
                    FieldElementCalculator fc;
                    auto pose3 = fc.CalcOffsetPositionForElement(visTagPose.value(), FieldConstants::FIELD_ELEMENT_OFFSETS::LEFT_STICK);
                    units::angle::degree_t fieldRelativeAngle = m_chassis->GetYaw() - pose3.ToPose2d().Rotation().Degrees();
                    DragonVisionStructLogger::logPose3d("Left Branch Vision", pose3);
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonTargetFinder", "Field Realitve Angle-Left", fieldRelativeAngle.to<double>());
                    m_goalPose = pose3.ToPose2d();

                    // return make_tuple(DragonTargetFinderData::VISION_BASED, pose3.ToPose2d());
                }

                // If no vision, then just use odometry based pose
                auto leftbranch = ReefHelper::GetInstance()->GetNearestLeftReefBranch(tag);
                if (leftbranch.has_value())
                {
                    auto leftbranchpose = fieldconst->GetFieldElementPose(leftbranch.value()).ToPose2d();
                    m_goalPose = leftbranchpose;
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, leftbranchpose);
                }
            }
            else // right branch
            {
                // TODO:  Update when we have reef machine learning
                //  Have a vision pose of the tag, calculate the offset to the reef branch
                if (m_switchToVision)
                {
                    FieldElementCalculator fc;
                    auto pose3 = fc.CalcOffsetPositionForElement(visTagPose.value(), FieldConstants::FIELD_ELEMENT_OFFSETS::RIGHT_STICK);
                    units::angle::degree_t fieldRelativeAngle = m_chassis->GetYaw() - pose3.ToPose2d().Rotation().Degrees();
                    DragonVisionStructLogger::logPose3d("Right Branch Vision", pose3);
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonTargetFinder", "Field Realitve Angle-Right", fieldRelativeAngle.to<double>());
                    m_goalPose = pose3.ToPose2d();
                    // return make_tuple(DragonTargetFinderData::VISION_BASED, pose3.ToPose2d());
                }

                // If no vision, then just use odometry based pose
                auto rightbranch = ReefHelper::GetInstance()->GetNearestRightReefBranch(tag);
                if (rightbranch.has_value())
                {
                    auto rightbranchpose = fieldconst->GetFieldElementPose(rightbranch.value()).ToPose2d();
                    m_goalPose = rightbranchpose;
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, rightbranchpose);
                }
            }
        }
    }
    else if (item == DragonTargetFinderTarget::REEF_CENTER)
    {
        auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
        if (allianceColor == frc::DriverStation::Alliance::kRed)
        {
            return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, fieldconst->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER).ToPose2d());
        }
        else
        {
            return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, fieldconst->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER).ToPose2d());
        }
    }
    else if (item == DragonTargetFinderTarget::BARGE)
    {
        auto bargeHelper = BargeHelper::GetInstance();
        if (bargeHelper != nullptr)
        {
            auto pose2d = bargeHelper->CalcBargePose();
            if (bargeHelper->ClampChassisY().has_value())
            {
                pose2d = frc::Pose2d(pose2d.X(), bargeHelper->ClampChassisY().value(), pose2d.Rotation());
            }
            else
            {
                pose2d = frc::Pose2d(pose2d.X(), m_chassis->GetPose().Y(), pose2d.Rotation());
            }
            return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, pose2d);
        }
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
            auto tagpose{fieldconst->GetAprilTagPose(tag)};
            if (item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE)
            {
                auto visiondata = m_vision->GetVisionData(DragonVision::VISION_ELEMENT::CORAL_STATION);
                if (visiondata.has_value())
                {
                    auto visiontagpose = GetVisonPose(visiondata.value());
                    if (visiontagpose)
                    {
                        if (visiontagpose.value().Translation().Distance(tagpose.ToPose2d().Translation()) < 1_m)
                        {
                            m_goalPose = visiontagpose.value();
                            return make_tuple(DragonTargetFinderData::VISION_BASED, visiontagpose.value());
                        }
                    }
                }

                return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, tagpose.ToPose2d());
            }
            else if (item == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE)
            {
                auto sidewall = CoralStationHelper::GetInstance()->GetNearestSideWallCoralStation(tag);
                if (sidewall.has_value())
                {
                    auto sidewallpose = fieldconst->GetFieldElementPose(sidewall.value()).ToPose2d();
                    m_goalPose = sidewallpose;
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, sidewallpose);
                }
            }
            else // CLOSEST_CORAL_STATION_ALLIANCE_SIDE
            {
                auto alliance = CoralStationHelper::GetInstance()->GetNearestAllianceWallCoralStation(tag);
                if (alliance.has_value())
                {
                    auto alliancepose = fieldconst->GetFieldElementPose(alliance.value()).ToPose2d();
                    m_goalPose = alliancepose;
                    return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, alliancepose);
                }
            }
        }
    }
    else if (item == DragonTargetFinderTarget::LEFT_CAGE ||
             item == DragonTargetFinderTarget::CENTER_CAGE ||
             item == DragonTargetFinderTarget::RIGHT_CAGE)
    {
        auto bargeHelper = BargeHelper::GetInstance();
        if (bargeHelper != nullptr)
        {
            auto cagepose = bargeHelper->GetCagePose(item);
            m_goalPose = cagepose;
            return make_tuple(DragonTargetFinderData::ODOMETRY_BASED, cagepose);
        }
    }

    auto pose2d = Pose2d();
    targetInfo = make_tuple(DragonTargetFinderData::NOT_FOUND, pose2d);
    return targetInfo;
}

std::optional<FieldConstants::AprilTagIDs> DragonTargetFinder::GetAprilTag(DragonVision::VISION_ELEMENT item)
{
    SetChassis();
    if (item == DragonVision::VISION_ELEMENT::REEF)
    {
        // call reef helper to find the appropriate closest side of the reef,
        // its corresponding APRILTAG ID and the field constant identifier
        return ReefHelper::GetInstance()->GetNearestReefTag();
    }
    else if (item == DragonVision::VISION_ELEMENT::CORAL_STATION)
    {
        // call coral station helper to find the appropriate the coral station,
        // its corresponding APRILTAG ID and the field constant identifier
        return CoralStationHelper::GetInstance()->GetNearestCoralStationTag();
    }
    else if (item == DragonVision::VISION_ELEMENT::ALGAE)
    {
        return std::nullopt; // TODO JW come back to this one when we have machine learning
    }
    else if (item == DragonVision::VISION_ELEMENT::BARGE)
    {
        return std::nullopt; // TODO JW come back to this one
    }
    else if (item == DragonVision::VISION_ELEMENT::PROCESSOR)
    {
        return std::nullopt; // TODO JW come back to this one
    }
    else
    {
        return std::nullopt; // TODO JW come back to this one
    }

    return std::nullopt;
}

frc::Pose3d DragonTargetFinder::GetAprilTagPose(DragonVision::VISION_ELEMENT item)
{
    auto aprilTag = GetAprilTag(item);
    if (aprilTag.has_value())
    {
        auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag.value());
        if (pose)
        {
            return pose.value();
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

std::optional<frc::Pose2d> DragonTargetFinder::GetVisonPose(VisionData data)
{
    SetChassis();

    if (m_chassis != nullptr)
    {
        auto currentPose{Pose3d(m_chassis->GetPose())};

        auto trans3d = data.transformToTarget;
        auto targetPose = currentPose + trans3d;
        units::angle::degree_t robotRelativeAngle = data.rotationToTarget.Z(); // value is robot to target

        units::angle::degree_t fieldRelativeAngle = currentPose.Rotation().Angle() + robotRelativeAngle;
        return frc::Pose2d(targetPose.X(), targetPose.Y(), fieldRelativeAngle);
    }
    return std::nullopt;
}

bool DragonTargetFinder::SwitchToVision(std::optional<frc::Pose3d> visTagPose) // TODO: Update when we switch to ML and raw vision correction on reef sticks
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonTargetFinder", "visTagPose has value", visTagPose.has_value() ? "true" : "false");

    if (visTagPose.has_value())
    {
        frc::Pose2d currentPose = m_chassis->GetPose();
        frc::Pose2d targetPose = visTagPose.value().ToPose2d();
        units::length::meter_t distanceToTarget = targetPose.Translation().Distance(currentPose.Translation());
        if (distanceToTarget < m_switchToVisionThreshold)
        {
            return true;
        }
    }
    return false;
}

void DragonTargetFinder::SetChassis()
{
    if (m_chassis == nullptr)
    {
        m_chassis = ChassisConfigMgr::GetInstance()->GetCurrentChassis();
    }
}

void DragonTargetFinder::DataLog(uint64_t timestamp)
{

    if (m_goalPose.has_value())
    {
        if (m_switchToVision)
        {
            if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE, m_goalPose.value());
            }
            else if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE, m_goalPose.value());
            }
            else if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE || m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE || m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE, m_goalPose.value());
            }
        }
        else
        {
            if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE, m_goalPose.value());
            }
            else if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE, m_goalPose.value());
            }
            else if (m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE || m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE || m_targetVisionTarget == DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE)
            {
                Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE, m_goalPose.value());
            }
        }
    }
}