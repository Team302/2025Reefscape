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
#include "chassis/CommandSwerveDrivetrain.h"
#include "RobotIdentifier.h"
#include "frc/RobotController.h"
#include "utils/logging/debug/Logger.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    RobotIdentifier id = static_cast<RobotIdentifier>(teamNumber);

    switch (id)
    {
    case RobotIdentifier::COMP_BOT_302:
        return TunerConstants302::CreateDrivetrain();
        break;

    case RobotIdentifier::CHASSIS_BOT_9998:
        return TunerConstants9998::CreateDrivetrain();
        break;

    case RobotIdentifier::SIM_BOT_0:
        return TunerConstants302::CreateDrivetrain();
        break;

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("Skipping chassis initialization because of unknown robot id "), std::string(""), static_cast<int>(id));
        break;
    }
}
