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
#include <cmath>
#include <string>

// Team302 Includes
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/states/MaintainHeading.h"
#include "utils/AngleUtils.h"

/// DEBUGGING
#include "utils/logging/debug/Logger.h"

using std::string;

MaintainHeading::MaintainHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::MAINTAIN)
{
    m_controller.EnableContinuousInput(-PI / 2.0, PI / 2.0);
    m_controller.SetIZone(0.174533); // 10 degrees in radians
}

std::string MaintainHeading::GetHeadingStateName() const
{
    return std::string("MaintainHeading");
}

void MaintainHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        if (!chassis->IsRotating())
        {
            auto correction = units::angular_velocity::degrees_per_second_t(0.0);

            auto currentAngle = units::angle::radian_t(AngleUtils::GetEquivAngle(chassis->GetPose().Rotation().Degrees()));
            auto targetAngle = units::angle::radian_t(AngleUtils::GetEquivAngle(chassis->GetStoredHeading()));

            auto radianCorrection = m_controller.Calculate(currentAngle.value(), targetAngle.value());

            correction = abs(radianCorrection) > m_correctionThreshold ? units::angular_velocity::radians_per_second_t(radianCorrection) : units::angular_velocity::radians_per_second_t(0.0);
            chassisMovement.chassisSpeeds.omega += correction;
        }
        else
            chassis->SetStoredHeading(chassis->GetYaw());
    }
}