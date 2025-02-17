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
#include <math.h>

#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "auton/AutonGrid.h"
#include "utils/logging/debug/Logger.h"

// Thirdparty includes

AutonGrid *AutonGrid::m_instance = nullptr; // initialize m_instance as a nullptr

AutonGrid *AutonGrid::GetInstance()
{
    // if m_instance is nullptr then a new instance of AutonGrid is created and returned therefore only leaving one instance of the class
    if (AutonGrid::m_instance == nullptr)
    {
        AutonGrid::m_instance = new AutonGrid();
    }
    return AutonGrid::m_instance;
} // to make the class a singlton

bool AutonGrid::IsPoseInZone(XGRID xgrid1, XGRID xgrid2, YGRID ygrid1, YGRID ygrid2, frc::Pose2d robotPose)
// defining IsPoseInZone bool method and pulling in the arguements
{
    // cast the enums xgrid1, etc to doubles
    double x1 = static_cast<double>((xgrid1));
    double y1 = static_cast<double>((ygrid1));
    double x2 = static_cast<double>((xgrid2));
    double y2 = static_cast<double>((ygrid2));

    // then it is determined wether or not the robotPose is in the zone defined by the 2 grids.
    return ((robotPose.X().value() >= units::length::meter_t(x1 * m_gridRes).value()) && (robotPose.X().value() <= units::length::meter_t(x2 * m_gridRes).value()) &&
            (robotPose.Y().value() >= units::length::meter_t(y1 * m_gridRes).value()) && (robotPose.Y().value() <= units::length::meter_t(y2 * m_gridRes).value()));
}
bool AutonGrid::IsPoseInZone(frc::Pose2d circleZonePose, units::length::inch_t radius, frc::Pose2d robotPose)
{
    auto translationToCenter = circleZonePose.Translation().Distance(robotPose.Translation());
    bool inZone = translationToCenter <= radius;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "circle zones", "in zone", inZone);
    return inZone;
}