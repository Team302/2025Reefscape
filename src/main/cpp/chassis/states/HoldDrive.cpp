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

// FRC Includes
#include "frc/geometry/Rotation2d.h"

// Team302 Includes
#include "chassis/states/HoldDrive.h"

HoldDrive::HoldDrive()
{
    m_flState->angle = {units::angle::degree_t(45)};
    m_frState->angle = {units::angle::degree_t(-45)};
    m_blState->angle = {units::angle::degree_t(135)};
    m_brState->angle = {units::angle::degree_t(-135)};
}

std::string HoldDrive::GetDriveStateName() const
{
    return std::string("HoldDrive");
}

std::array<frc::SwerveModuleState, 4> HoldDrive::UpdateSwerveModuleStates(
    ChassisMovement &chassisMovement)
{
    return {*m_flState, *m_frState, *m_blState, *m_brState};
}

void HoldDrive::Init(
    ChassisMovement &chassisMovement)
{
}