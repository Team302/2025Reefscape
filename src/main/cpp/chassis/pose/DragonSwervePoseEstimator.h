#pragma once
#include <vector>
#include "chassis/ChassisConfigMgr.h"
#include "chassis/pose/DragonVisionPoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

class DragonSwervePoseEstimator
{
public:
    DragonSwervePoseEstimator();
    ~DragonSwervePoseEstimator() = default;

    void Update();

    void RegisterVisionPoseEstimator(DragonVisionPoseEstimator *poseEstimator);

    void ResetPosition(const frc::Pose2d &pose);
    frc::Pose2d GetPose() const;

private:
    void AddVisionMeasurements();

    subsystems::CommandSwerveDrivetrain *m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();

    std::vector<DragonVisionPoseEstimator *> m_visionPoseEstimators;
};