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
#include "chassis/states/DriveToTarget.h" // Update path if needed
#include "utils/AngleUtils.h"
#include "utils/logging/debug/Logger.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"

DriveToTarget::DriveToTarget(
    subsystems::CommandSwerveDrivetrain *chassis,
    DragonTargetFinderTarget target) : m_chassis(chassis),
                                       m_target(target),
                                       m_targetFinder(DragonTargetFinder::GetInstance()),
                                       m_hasTarget(false),
                                       m_samePoseCount(0),
                                       m_translationPIDX(4.5, 0.0, 0.0, {kMaxVelocity, kMaxAcceleration}),
                                       m_translationPIDY(4.5, 0.0, 0.0, {kMaxVelocity, kMaxAcceleration})
{
    AddRequirements(m_chassis);
}

void DriveToTarget::Initialize()
{
    m_hasTarget = false;
    m_samePoseCount = 0;

    if (m_targetFinder)
    {
        m_targetFinder->ResetGoalPose();
        auto info = m_targetFinder->GetPose(m_target);
        if (info.has_value())
        {
            m_hasTarget = std::get<0>(info.value()) != DragonTargetFinderData::NOT_FOUND;
            if (m_hasTarget)
            {
                m_targetPose = std::get<1>(info.value());

                auto currentPose = m_chassis->GetState().Pose;
                m_translationPIDX.Reset(currentPose.X());
                m_translationPIDY.Reset(currentPose.Y());
            }
        }
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToTarget", "Target X", m_targetPose.X().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToTarget", "Target Y", m_targetPose.Y().value());

    if (!m_hasTarget)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "DriveToTarget", "Target not found on init", (int)m_target);
    }
}

void DriveToTarget::Execute()
{
    if (m_chassis != nullptr)
    {

        // Update target pose if vision provides a better estimate
        auto info = m_targetFinder->GetPose(m_target);
        if (info.has_value() && std::get<0>(info.value()) != DragonTargetFinderData::NOT_FOUND)
        {
            m_targetPose = std::get<1>(info.value());
        }

        auto currentPose = m_chassis->GetState().Pose;

        // Calculate feed-forward velocity
        units::meters_per_second_t xSpeed_ff{0.0};
        units::meters_per_second_t ySpeed_ff{0.0};
        CalculateFeedForward(xSpeed_ff, ySpeed_ff);

        // Calculate PID feedback velocity
        units::meters_per_second_t xSpeed_pid{m_translationPIDX.Calculate(currentPose.X(), m_targetPose.X())};
        units::meters_per_second_t ySpeed_pid{m_translationPIDY.Calculate(currentPose.Y(), m_targetPose.Y())};

        // Combine feed-forward and PID
        units::meters_per_second_t targetXSpeed = xSpeed_ff + xSpeed_pid;
        units::meters_per_second_t targetYSpeed = ySpeed_ff + ySpeed_pid;

        // Set the robot's heading to face the target

        m_chassis->SetControl(
            m_driveRequest.WithVelocityX(xSpeed_ff)
                .WithVelocityY(ySpeed_ff)
                .WithTargetDirection(m_targetPose.Rotation().Degrees()));
    }
}

bool DriveToTarget::IsFinished()
{
    if (!m_hasTarget)
    {
        return true; // End immediately if we never had a target
    }

    auto currentPose = m_chassis->GetState().Pose;
    auto distance = currentPose.Translation().Distance(m_targetPose.Translation());

    // Check if we are close enough to the target
    if (distance < m_distanceThreshold)
    {
        m_samePoseCount++;
    }
    else
    {
        m_samePoseCount = 0;
    }

    // End if we have been at the target for a few cycles
    return m_samePoseCount > 10;
}

void DriveToTarget::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}

void DriveToTarget::CalculateFeedForward(units::meters_per_second_t &vx, units::meters_per_second_t &vy)
{
    auto currentPose = m_chassis->GetState().Pose;
    units::meter_t distanceError = currentPose.Translation().Distance(m_targetPose.Translation());
    units::meter_t feedForwardRange = m_ffMaxRadius - m_ffMinRadius;

    // Calculate feedforward speed based on distance
    units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
    if (distanceError > m_ffMinRadius && feedForwardRange.value() > 0)
    {
        double feedForwardScale = std::clamp(((distanceError - m_ffMinRadius) / (feedForwardRange)).value(), 0.0, 1.0);
        feedforwardSpeed = kMaxVelocity * feedForwardScale;
    }

    // Apply feedforward to the desired velocity
    frc::Translation2d translationError = m_targetPose.Translation() - currentPose.Translation();
    frc::Rotation2d angleToTarget = translationError.Angle();

    vx = feedforwardSpeed * angleToTarget.Cos();
    vy = feedforwardSpeed * angleToTarget.Sin();
}
