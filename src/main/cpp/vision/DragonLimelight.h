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

// C++ Includes
#include <string>
#include <vector>

// FRC includes
#include "frc/Timer.h"
#include "networktables/NetworkTable.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "frc/geometry/Pose2d.h"

// Team 302 includes
#include "vision/DragonCamera.h"
#include "utils/sensors/SensorData.h"

// Third Party Includes

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight : public DragonCamera, public SensorData
{
public:
    enum LED_MODE
    {
        LED_UNKNOWN = -1,
        LED_PIPELINE_CONTROL,
        LED_OFF,
        LED_BLINK,
        LED_ON
    };

    enum CAM_MODE
    {
        CAM_UNKNOWN = -1,
        CAM_VISION,
        CAM_DRIVER
    };

    enum STREAM_MODE
    {
        STREAM_UNKNOWN = -1,
        STREAM_STANDARD,         // side by side if two cams
        STREAM_PIP_MAIN, // Second Cam bottom right of Main Cam
        STREAM_PIP_SECONDARY  // Main Cam bottom right of Second Cam
    };

    enum SNAPSHOT_MODE
    {
        SNAPSHOT_MODE_UNKNOWN = -1,
        SNAP_OFF,
        SNAP_ON
    };

    enum LL_PIPELINE
    {
        UNKNOWN = -1,
        OFF,
        MACHINE_LEARNING_PL,
        APRIL_TAG,
        COLOR_THRESHOLD
    };

    ///-----------------------------------------------------------------------------------
    /// Method:         DragonLimelight (constructor)
    /// Description:    Create the object
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;
    DragonLimelight(
        std::string name, /// <I> - network table name'
        DragonCamera::CAMERA_TYPE cameraType,
        DragonCamera::CAMERA_USAGE cameraUsage,
        units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
        units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
        units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
        units::angle::degree_t pitch,          /// <I> - Pitch of camera
        units::angle::degree_t yaw,            /// <I> - Yaw of camera
        units::angle::degree_t roll,           /// <I> - Roll of camera
        LL_PIPELINE initialPipeline,           /// <I> enum for starting pipeline
        LED_MODE ledMode,
        CAM_MODE camMode,
        STREAM_MODE streamMode,
        SNAPSHOT_MODE snapMode);

    ///-----------------------------------------------------------------------------------
    /// Method:         ~DragonLimelight (destructor)
    /// Description:    Delete the object
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    bool HealthCheck();
    bool HasTarget();

    virtual std::optional<units::angle::degree_t> GetTargetYaw();
    std::optional<units::angle::degree_t> GetTargetYawRobotFrame();
    virtual std::optional<units::angle::degree_t> GetTargetPitch();
    std::optional<units::angle::degree_t> GetTargetPitchRobotFrame();
    std::optional<double> GetTargetArea();
    std::optional<units::angle::degree_t> GetTargetSkew();
    std::optional<units::time::millisecond_t> GetPipelineLatency();
    std::optional<int> GetAprilTagID();

    std::vector<double> Get3DSolve();

    std::optional<VisionPose> GetFieldPosition();
    std::optional<VisionPose> GetFieldPosition(frc::DriverStation::Alliance alliance);

    std::optional<VisionPose> EstimatePoseOdometryLimelight(bool megatag2);

    std::optional<VisionPose> GetRedFieldPosition();
    std::optional<VisionPose> GetBlueFieldPosition();
    std::optional<VisionPose> GetOriginFieldPosition();

    std::optional<VisionData> GetDataToNearestAprilTag();
    std::optional<VisionData> GetDataToSpecifiedTag(int id);

    std::optional<units::length::inch_t> EstimateTargetXDistance();
    std::optional<units::length::inch_t> EstimateTargetYDistance();
    std::optional<units::length::inch_t> EstimateTargetZDistance();

    std::optional<units::length::inch_t> EstimateTargetXDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> EstimateTargetYDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> EstimateTargetZDistance_RelToRobotCoords();

    units::length::inch_t CalcXTargetToRobot(units::angle::degree_t camPitch, units::length::inch_t mountHeight, units::length::inch_t camXOffset, units::angle::degree_t tY);
    units::length::inch_t CalcYTargetToRobot(units::angle::degree_t camYaw, units::length::inch_t xTargetDistance, units::length::inch_t camYOffset, units::length::inch_t camXOffset, units::angle::degree_t tX);

    // limelight specific helper functions
    void SetLEDMode(DragonLimelight::LED_MODE mode);
    void SetCamMode(DragonLimelight::CAM_MODE mode);
    void SetPipeline(DragonLimelight::LL_PIPELINE pipeline);
    void SetStreamMode(DragonLimelight::STREAM_MODE mode);
    void ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle);
    void SetCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetSecondaryCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetPriorityTagID(int id);
    void SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw);

    void PeriodicCacheData() override;

    

    void PrintValues(); // Prints out all values to ensure everything is working and connected

protected:
    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;
    units::length::inch_t m_driveThroughOffset = units::length::inch_t(0.0);

    std::shared_ptr<nt::NetworkTable> m_networktable;

    bool m_tv;
    units::angle::degree_t m_tx;
    units::angle::degree_t m_ty;

    const double START_HB = -9999;
    const double MAX_HB = 2000000000;
    double m_lastHeartbeat = START_HB;
    frc::Timer *m_healthTimer;
    LL_PIPELINE m_pipeline;
};
