
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

// FRC includesssss

// Team 302 includes
#include "auton/drivePrimitives/DriveStopMech.h"
#include "auton/drivePrimitives/DriveStop.h"
#include "auton/PrimitiveParams.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
// #include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace frc;

//========================================================================================================
/// @class  DriveStop
/// @brief  This is an auton primitive that causes the chassis to not drive
//========================================================================================================

/// @brief constructor that creates/initializes the object
DriveStopMech::DriveStopMech() : DriveStop()
{
    auto dragonTale = MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::DRAGON_TALE);
    m_dragonTaleMgr = dragonTale != nullptr ? dynamic_cast<DragonTale *>(dragonTale) : nullptr;
}
void DriveStopMech::Init(PrimitiveParams *params)
{
    DriveStop::Init(params);
    m_switchState = params->GetTaleState() == DragonTale::STATE_NAMES::STATE_SCORE_CORAL ? DragonTale::STATE_NAMES::STATE_READY : DragonTale::STATE_NAMES::STATE_HOLD;
}

/// @brief check if the end condition has been met
/// @return bool true means the end condition was reached, false means it hasn't
bool DriveStopMech::IsDone()
{
    return m_dragonTaleMgr->GetCurrentState() == m_switchState || DriveStop::IsDone();
}
