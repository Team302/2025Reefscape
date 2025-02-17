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

#include <string>

// FRC Includes
#include "frc/Filesystem.h"
#include "frc/trajectory/TrajectoryUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

// Team302 Includes
#include "auton/AutonPreviewer.h"
#include "auton/AutonSelector.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "chassis/SwerveModule.h"

// Thirdparty includes
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace pathplanner;
using frc::ChassisSpeeds;
using frc::Rotation2d;
using frc::Trajectory;
using std::string;

AutonPreviewer::AutonPreviewer(CyclePrimitives *cyclePrims) : m_selector(cyclePrims->GetAutonSelector()),
                                                              m_prevChoice(""),
                                                              m_field(DragonField::GetInstance())
{
}

void AutonPreviewer::CheckCurrentAuton()
{
    std::string currentChoice = m_selector->GetSelectedAutoFile();
    if (currentChoice != m_prevChoice)
    {
        PopulateField();
        m_prevChoice = currentChoice;
    }
}

void AutonPreviewer::PopulateField()
{
    auto trajectories = GetTrajectories();
    m_field->ResetField();
    for (unsigned int i = 0; i < trajectories.size(); i++)
    {
        m_field->AddTrajectory("traj" + std::to_string(i), trajectories[i]);
    }
}

std::vector<frc::Trajectory> AutonPreviewer::GetTrajectories()
{

    std::vector<frc::Trajectory> trajectories;

    ChassisSpeeds speeds;
    speeds.vx = units::velocity::feet_per_second_t(0.0);
    speeds.vy = units::velocity::feet_per_second_t(0.0);
    speeds.omega = units::angular_velocity::degrees_per_second_t(0.0);

    Rotation2d heading(units::angle::degree_t(0.0));

    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        auto swMod = chassis->GetFrontLeft();
        if (swMod != nullptr)
        {
            auto moduleConfig = ModuleConfig(swMod->GetWheelDiameter() / 2.0,
                                             swMod->GetMaxSpeed(),
                                             swMod->GetCoefficientOfFriction(),
                                             swMod->GetDriveMotorDef(),
                                             swMod->GetDriveCurrentLimit(), 1);
            auto robotConfig = pathplanner::RobotConfig(chassis->GetMass(), chassis->GetMomenOfInertia(), moduleConfig, chassis->GetTrack());

            auto params = PrimitiveParser::ParseXML(m_selector->GetSelectedAutoFile());

            for (auto param : params)
            {
                std::vector<Trajectory::State> states;

                if (param->GetID() == PRIMITIVE_IDENTIFIER::DRIVE_PATH_PLANNER && false)
                {
                    auto pathname = param->GetPathName();
                    auto path = AutonUtils::GetPathFromPathFile(pathname);
                    if (AutonUtils::IsValidPath(path))
                    {
                        auto trajectory = path.get()->generateTrajectory(speeds, chassis->GetPose().Rotation(), robotConfig);
                        auto endstate = trajectory.getEndState();
                        heading = endstate.heading;
                        speeds.vx = endstate.linearVelocity * heading.Cos();
                        speeds.vy = endstate.linearVelocity * heading.Sin();

                        auto ppstates = trajectory.getStates();
                        for (auto ppstate : ppstates)
                        {
                            Trajectory::State state;
                            state.t = ppstate.time;
                            state.acceleration = ppstate.constraints.getMaxAcceleration();
                            state.velocity = ppstate.linearVelocity;
                            state.pose = ppstate.pose;
                            state.curvature = units::curvature_t(0.1); // ppstate.constraints. curvature;

                            states.emplace_back(state);
                        }
                    }
                }
            }
        }
    }
    return trajectories;
}