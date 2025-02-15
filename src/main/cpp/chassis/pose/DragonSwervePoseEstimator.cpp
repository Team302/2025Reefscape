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

#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/pose/DragonVisionPoseEstimator.h"
#include "chassis/SwerveChassis.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "units/time.h"
#include "wpi/array.h"
#include "vision/DragonVision.h"

DragonSwervePoseEstimator::DragonSwervePoseEstimator(frc::SwerveDriveKinematics<4> kinematics,
                                                     const frc::Rotation2d &gyroAngle,
                                                     const wpi::array<frc::SwerveModulePosition, 4> &positions,
                                                     const frc::Pose2d &initialPose) : // m_chassis(chassis),
                                                                                       m_frontLeft(nullptr),
                                                                                       m_frontRight(nullptr),
                                                                                       m_backLeft(nullptr),
                                                                                       m_backRight(nullptr),
                                                                                       m_kinematics(kinematics),
                                                                                       m_poseEstimator(kinematics, gyroAngle, positions, initialPose),
                                                                                       m_visionPoseEstimators()
{
    SetServeModules(ChassisConfigMgr::GetInstance()->GetCurrentChassis());
}

void DragonSwervePoseEstimator::SetServeModules(SwerveChassis *chassis)
{
    if (chassis != nullptr)
    {
        m_frontLeft = chassis->GetFrontLeft();
        m_frontRight = chassis->GetFrontRight();
        m_backLeft = chassis->GetBackLeft();
        m_backRight = chassis->GetBackRight();
    }
}

void DragonSwervePoseEstimator::RegisterVisionPoseEstimator(DragonVisionPoseEstimator *poseEstimator)
{
    m_visionPoseEstimators.push_back(poseEstimator);
}

void DragonSwervePoseEstimator::Update()
{

    auto chassis = ChassisConfigMgr::GetInstance()->GetCurrentChassis();
    if (chassis != nullptr)
    {
        if (m_frontLeft == nullptr) // Ensure there isn't a timing issue
        {
            SetServeModules(chassis);
        }
        frc::Rotation2d rot2d{chassis->GetRawYaw()};

        m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(),
                                                                               m_frontRight->GetPosition(),
                                                                               m_backLeft->GetPosition(),
                                                                               m_backRight->GetPosition()});
    }
    // AddVisionMeasurements();

    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::ChassisPose_Pose2D, GetPose());
}

void DragonSwervePoseEstimator::AddVisionMeasurements()
{
    for (auto estimator : m_visionPoseEstimators)
    {
        auto poseInfo = estimator->GetPoseEstimate();
        if (poseInfo.m_confidenceLevel == DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE)
        {
            continue;
        }

        // currently just using std deviation for vision measurements passed in;
        // may need to revisit if the different systems provide different std deviation values
        // for similar data confidence
        m_poseEstimator.AddVisionMeasurement(poseInfo.m_visionPose,
                                             poseInfo.m_timeStamp,
                                             poseInfo.m_stds);
    }
}

void DragonSwervePoseEstimator::ResetPosition(const frc::Pose2d &pose)
{
    auto chassis = ChassisConfigMgr::GetInstance()->GetCurrentChassis();
    if (chassis != nullptr && m_frontLeft != nullptr && m_frontRight != nullptr && m_backLeft != nullptr && m_backRight != nullptr)
    {
        auto yaw = chassis->GetYaw();

        m_poseEstimator.ResetPosition(yaw,
                                      wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(), m_frontRight->GetPosition(), m_backLeft->GetPosition(), m_backRight->GetPosition()},
                                      pose);
    }
}
void DragonSwervePoseEstimator::CalculateInitialPose()
{
    auto chassis = ChassisConfigMgr::GetInstance()->GetCurrentChassis();
    if (chassis != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();
        std::optional<frc::Pose2d> visionpose = vision->CalcVisionPose();
        if (visionpose != std::nullopt) // may want to use reset Position instead of reset pose here?
        {
            chassis->ResetPose(visionpose.value());
        }
    }
}
frc::Pose2d DragonSwervePoseEstimator::GetPose() const
{
    return m_poseEstimator.GetEstimatedPosition();
}

void DragonSwervePoseEstimator::ResetPose(const frc::Pose2d &pose)
{
    m_poseEstimator.ResetPose(pose);
}
