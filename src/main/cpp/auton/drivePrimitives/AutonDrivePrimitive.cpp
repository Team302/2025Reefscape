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
#include "auton/drivePrimitives/AutonDrivePrimitive.h"
#include "auton/PrimitiveParams.h"
#include "chassis/SwerveContainer.h"
#include "chassis/states/TrajectoryDrive.h"
#include "chassis/states/DriveToTarget.h"
#include "utils/logging/debug/Logger.h"
#include <frc2/command/Commands.h>
#include <frc2/command/ProxyCommand.h>

AutonDrivePrimitive::AutonDrivePrimitive() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                             m_timer(std::make_unique<frc::Timer>()),
                                             m_managedCommand(frc2::cmd::None())
{
}

void AutonDrivePrimitive::Init(PrimitiveParams *params)
{
    // Always start with a clean slate
    frc2::CommandScheduler::GetInstance().CancelAll();
    m_visionTransition = false;
    m_maxTime = params->GetTime();
    m_timer->Reset();
    m_timer->Start();
    m_zone = nullptr;

    switch (params->GetID())
    {
    case PRIMITIVE_IDENTIFIER::TRAJECTORY_DRIVE:
    {
        auto container = SwerveContainer::GetInstance();
        auto trajectoryDriveCmd = container->GetTrajectoryDriveCommand();
        trajectoryDriveCmd->SetPath(params->GetTrajectoryName()); // Set the path for this run

        m_managedCommand = frc2::ProxyCommand(trajectoryDriveCmd).ToPtr();

        // Check if this path has a vision update zone
        int index = FindDriveToZoneIndex(params->GetZones()); // Assume FindDriveToZoneIndex is moved here
        if (index != -1)
        {
            m_zone = params->GetZones()[index];
            m_checkForDriveToUpdate = (m_zone != nullptr);
        }
        break;
    }
    case PRIMITIVE_IDENTIFIER::DO_NOTHING:
    case PRIMITIVE_IDENTIFIER::DO_NOTHING_DELAY:
    case PRIMITIVE_IDENTIFIER::DO_NOTHING_MECHANISMS:
    case PRIMITIVE_IDENTIFIER::HOLD_POSITION:
    {
        m_managedCommand = frc2::cmd::RunOnce([this]()
                                              { m_chassis->SetControl(swerve::requests::SwerveDriveBrake{}); }, {m_chassis});
        break;
    }

    default:
        m_managedCommand = frc2::cmd::None();
        break;
    }

    if (m_managedCommand)
    {
        m_managedCommand.Schedule();
    }
}

void AutonDrivePrimitive::Run()
{
    // Logic to transition from trajectory to vision drive mid-path
    if (!m_visionTransition && m_checkForDriveToUpdate && IsInZone())
    {
        ChassisOptionEnums::DriveStateType driveToType = m_zone->GetPathUpdateOption();
        m_managedCommand = CreateDriveToTargetCommand(driveToType);
        if (m_managedCommand)
        {
            m_managedCommand.Schedule();
            m_visionTransition = true;
        }
    }
}

bool AutonDrivePrimitive::IsDone()
{
    bool timeout = m_timer->HasElapsed(m_maxTime);
    bool commandFinished = !m_managedCommand.IsScheduled();

    return timeout || commandFinished;
}

int AutonDrivePrimitive::FindDriveToZoneIndex(ZoneParamsVector zones)
{
    if (!zones.empty())
    {
        for (unsigned int i = 0; i < zones.size(); i++)
        {
            if (zones[i]->GetPathUpdateOption() != ChassisOptionEnums::DriveStateType::STOP_DRIVE)
            {
                return i;
            }
        }
    }
    return -1;
}

bool AutonDrivePrimitive::IsInZone()
{
    if (m_zone != nullptr && m_chassis != nullptr)
    {
        return m_zone->IsPoseInZone(m_chassis->GetPose());
    }
    return false;
}

frc2::CommandPtr AutonDrivePrimitive::CreateDriveToTargetCommand(ChassisOptionEnums::DriveStateType driveToType)
{
    auto container = SwerveContainer::GetInstance();
    switch (driveToType)
    {
    case ChassisOptionEnums::DRIVE_TO_CORAL_STATION:
        return frc2::ProxyCommand(container->GetDriveToCoralStationCommand()).ToPtr();

    case ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH:
        return frc2::ProxyCommand(container->GetDriveToCoralRightBranchCommand()).ToPtr();

    case ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH:
        return frc2::ProxyCommand(container->GetDriveToCoralLeftBranchCommand()).ToPtr();

    case ChassisOptionEnums::DRIVE_TO_BARGE:
        return frc2::ProxyCommand(container->GetDriveToBargeCommand()).ToPtr();

    default:
        return frc2::cmd::None();
    }
}