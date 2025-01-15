//====================================================================================================================================================
/// Copyright 2025 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// FRC includes
#include "frc/geometry/Rotation3d.h"

// Team 302 includes
#include "SensorData.h"
#include "SensorDataMgr.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

bool m_tv;
units::angle::degree_t m_tx;
units::angle::degree_t m_ty;

SensorData::SensorData(std::string networkTableName) : m_networktable(nt::NetworkTableInstance::GetDefault().GetTable(std::string(networkTableName)))
{
    SensorDataMgr::GetInstance()->RegisterSensorData(this);
}
void SensorData::PeriodicCacheData()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {

        m_tv = (nt->GetNumber("tv", 0.0) > 0.1);
        m_tx = units::angle::degree_t(nt->GetNumber("tx", 0.0));
        m_ty = units::angle::degree_t(nt->GetNumber("ty", 0.0));
    }
    else
    {
        m_tv = false;
        m_tx = units::angle::degree_t(0.0);
        m_ty = units::angle::degree_t(0.0);
    }
}
