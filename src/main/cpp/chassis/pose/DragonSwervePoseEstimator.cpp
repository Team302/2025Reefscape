#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/ChassisConfigMgr.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"

DragonSwervePoseEstimator::DragonSwervePoseEstimator() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()), // Get the pointer once
                                                         m_visionPoseEstimators()
{
}

void DragonSwervePoseEstimator::RegisterVisionPoseEstimator(DragonVisionPoseEstimator *poseEstimator)
{
    if (poseEstimator != nullptr)
    {
        m_visionPoseEstimators.push_back(poseEstimator);
    }
}

// This is the new main loop method
void DragonSwervePoseEstimator::Update()
{
    if (m_chassis != nullptr)
    {
        AddVisionMeasurements();
    }
}

void DragonSwervePoseEstimator::AddVisionMeasurements()
{
    // This logic is mostly the same, but it calls the chassis method.
    for (auto estimator : m_visionPoseEstimators)
    {
        // "Pull" the data from the vision system
        auto poseInfo = estimator->GetPoseEstimate();
        if (poseInfo.m_confidenceLevel != DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE)
        {
            // "Push" the data to the chassis's internal estimator
            m_chassis->AddVisionMeasurement(poseInfo.m_visionPose,
                                            poseInfo.m_timeStamp,
                                            poseInfo.m_stds);
        }
    }
}

frc::Pose2d DragonSwervePoseEstimator::GetPose() const
{
    return (m_chassis != nullptr) ? m_chassis->GetPose() : frc::Pose2d{};
}

void DragonSwervePoseEstimator::ResetPosition(const frc::Pose2d &pose)
{
    if (m_chassis != nullptr)
    {
        m_chassis->ResetPose(pose);
    }
}