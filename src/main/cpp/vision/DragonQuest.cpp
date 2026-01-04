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
#include "units/time.h"
#include "utils/AngleUtils.h"
#include "utils/DragonField.h"
#include "vision/DragonQuest.h"
#include "state/RobotStateChanges.h"
#include "state/RobotState.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "utils/logging/debug/Logger.h"

DragonQuest::DragonQuest(
    units::length::inch_t mountingXOffset, /// <I> x offset of Quest from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset, /// <I> y offset of Quest from robot center (left relative to robot)
    units::length::inch_t mountingZOffset, /// <I> z offset of Quest from robot center (up relative to robot)
    units::angle::degree_t mountingPitch,  /// <I> - Pitch of Quest
    units::angle::degree_t mountingYaw,    /// <I> - Yaw of Quest
    units::angle::degree_t mountingRoll    /// <I> - Roll of Quest
    ) : IRobotStateChangeSubscriber(),
        m_mountingXOffset(mountingXOffset),
        m_mountingYOffset(mountingYOffset),
        m_mountingZOffset(mountingZOffset),
        m_mountingPitch(mountingPitch),
        m_mountingYaw(mountingYaw),
        m_mountingRoll(mountingRoll)
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("QuestNav"));

    // Initialize protobuf topics
    m_frameDataSubscriber = m_networktable.get()->GetRawTopic("frameData").Subscribe("proto:questnav.protos.data.ProtobufQuestNavFrameData", {});
    m_deviceDataSubscriber = m_networktable.get()->GetRawTopic("deviceData").Subscribe("proto:questnav.protos.data.ProtobufQuestNavDeviceData", {});
    m_commandPublisher = m_networktable.get()->GetRawTopic("commands").Publish("proto:questnav.protos.commands.ProtobufQuestNavCommand");
    m_commandResponseSubscriber = m_networktable.get()->GetRawTopic("response").Subscribe("proto:questnav.protos.commands.ProtobufQuestNavCommandResponse", {});

    m_questToRobotTransform = frc::Transform2d{
        frc::Translation2d(m_mountingXOffset, m_mountingYOffset),
        frc::Rotation2d(m_mountingYaw)};

    m_questEnabledChooser.AddOption("ON", true);
    m_questEnabledChooser.AddOption("OFF", false);
    m_questEnabledChooser.SetDefaultOption("ON", true);

    m_questEndgameEnabledChooser.AddOption("ENDGAME ONLY", true);
    m_questEndgameEnabledChooser.AddOption("FULL MATCH", false);
    m_questEndgameEnabledChooser.SetDefaultOption("FULL MATCH", false);

    frc::SmartDashboard::PutData("Quest ON/OFF", &m_questEnabledChooser);
    frc::SmartDashboard::PutData("Quest Endgame ONLY", &m_questEndgameEnabledChooser);
    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
}

frc::Pose2d DragonQuest::GetEstimatedPose()
{
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        return frc::Pose2d{}; // Return default pose if no data
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonQuest"), string("GetEstimatedPose"), string("Failed to parse frame data"));
        return frc::Pose2d{};
    }

    if (!frameData.has_pose3d())
    {
        return frc::Pose2d{};
    }

    const auto &pose3d = frameData.pose3d();
    const auto &translation = pose3d.translation();
    const auto &rotation = pose3d.rotation();

    // Extract quaternion
    const auto &q = rotation.q();
    double qw = q.w();
    double qz = q.z();

    // Convert quaternion to yaw (rotation around Z-axis)
    // For a quaternion representing rotation around Z: yaw = 2 * atan2(qz, qw)
    double yawRadians = 2.0 * std::atan2(qz, qw);

    // Convert from Quest coordinates to robot coordinates
    units::length::meter_t x{translation.x()};
    units::length::meter_t y{-translation.y()};
    units::angle::radian_t yaw{-yawRadians};

    frc::Pose2d questPose{x, y, yaw};
    frc::Pose2d robotPose = questPose.TransformBy(m_questToRobotTransform.Inverse());

    return robotPose;
}

void DragonQuest::SetIsConnected()
{
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        m_isConnected = false;
        return;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        m_isConnected = false;
        return;
    }

    int32_t currentFrameCount = frameData.frame_count();

    if (m_loopCounter > 3)
    {
        if (currentFrameCount != m_prevFrameCount && frameData.istracking())
        {
            m_loopCounter = 0;
            m_isConnected = true;
        }
        else
        {
            m_isConnected = false;
        }
        m_prevFrameCount = currentFrameCount;
    }
    m_isConnected = true;
    m_loopCounter = (m_loopCounter > 3) ? 0 : m_loopCounter + 1;
}

void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, GetEstimatedPose());
}

void DragonQuest::HandleHeartBeat()
{
    // if (m_questMiso.Get() == 98)
    // {
    //     m_questMosi.Set(0);
    // }
    // double requestId = m_heartbeatRequestSub.Get();
    // // Only respond to new requests to avoid flooding
    // if (requestId > 0 && requestId != m_lastProcessedHeartbeatId)
    // {
    //     m_heartbeatResponsePub.Set(requestId);
    //     m_lastProcessedHeartbeatId = requestId;
    // }
    SetIsConnected();
}

void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    if (!m_hasReset)
    {
        frc::Pose2d questPose = pose.TransformBy(m_questToRobotTransform);

        // Create pose reset command
        questnav::protos::commands::ProtobufQuestNavCommand command;
        command.set_type(questnav::protos::commands::POSE_RESET);
        command.set_command_id(m_nextCommandId++);

        auto *payload = command.mutable_pose_reset_payload();
        auto *targetPose = payload->mutable_target_pose();

        // Set translation
        auto *translation = targetPose->mutable_translation();
        translation->set_x(questPose.X().value());
        translation->set_y(questPose.Y().value());
        translation->set_z(0.0);

        // Set rotation using quaternion
        auto *rotation = targetPose->mutable_rotation();
        auto *quaternion = rotation->mutable_q();

        // Convert 2D rotation to quaternion (rotation around Z axis)
        double yawRadians = questPose.Rotation().Radians().value();
        double halfYaw = yawRadians / 2.0;

        quaternion->set_w(std::cos(halfYaw)); // Real part
        quaternion->set_x(0.0);               // X axis
        quaternion->set_y(0.0);               // Y axis
        quaternion->set_z(std::sin(halfYaw)); // Z axis (yaw rotation)

        // Serialize and publish
        std::string serialized;
        command.SerializeToString(&serialized);
        m_commandPublisher.Set(std::span<const uint8_t>(
            reinterpret_cast<const uint8_t *>(serialized.data()),
            serialized.size()));

        m_hasReset = true;
    }
}

void DragonQuest::HandleDashboard()
{

    if (m_questEnabledChooser.GetSelected() == true)
    {
        m_isQuestEnabled = true;
        if (m_questEndgameEnabledChooser.GetSelected() == true && m_climbMode != RobotStateChanges::ClimbMode::ClimbModeOn)
        {
            m_isQuestEnabled = false;
        }
    }
    else
    {
        m_isQuestEnabled = false;
    }
}

void DragonQuest::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Int == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
}
DragonVisionPoseEstimatorStruct DragonQuest::GetPoseEstimate()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("m_hasReset"), m_hasReset);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("m_isConnected"), m_isConnected);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("m_isQuestEnabled"), m_isQuestEnabled);

    DragonVisionPoseEstimatorStruct str;
    HandleDashboard();
    if (!m_hasReset || !m_isConnected || !m_isQuestEnabled)
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("confidence"), string("NONE"));
    }
    else
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH;
        str.m_visionPose = GetEstimatedPose();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("x"), str.m_visionPose.X().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("y"), str.m_visionPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("rot"), str.m_visionPose.Rotation().Degrees().value());
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
        // str.m_timeStamp = units::time::second_t(m_timestamp.GetAtomic().serverTime);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("confidence"), string("HIGH"));
    }
    return str;
}
