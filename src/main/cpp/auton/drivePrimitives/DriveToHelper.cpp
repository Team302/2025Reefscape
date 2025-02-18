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

// Team 302 includes
#include "auton/drivePrimitives/DriveToHelper.h"
#include "chassis/states/DriveToRightReefBranch.h"
#include "chassis/states/DriveToLeftReefBranch.h"
#include "chassis/states/DriveToCoralStation.h"
#include "auton/PrimitiveParams.h"

DriveToHelper::DriveToHelper(SwerveChassis *chassis, ChassisMovement moveInfo) : m_driveTo(nullptr),
                                                                                 m_chassis(chassis),
                                                                                 m_moveInfo(moveInfo)
{
}

void DriveToHelper::Init(UPDATE_OPTION pathUpdateOption)
{
    if (m_chassis != nullptr && pathUpdateOption != UPDATE_OPTION::NOTHING)
    {
        switch (pathUpdateOption)
        {
        case RIGHT_REEF_BRANCH:
            m_driveTo = dynamic_cast<DriveToRightReefBranch *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH));
            break;
        case LEFT_REEF_BRANCH:
            m_driveTo = dynamic_cast<DriveToLeftReefBranch *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH));
            break;
        case CORAL_STATION:
            m_driveTo = dynamic_cast<DriveToCoralStation *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DRIVE_TO_CORAL_STATION));
        }
    }
}
