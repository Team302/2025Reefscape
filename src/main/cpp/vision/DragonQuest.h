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

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include "networktables/DoubleArrayTopic.h"
#include "utils/logging/DragonDataLogger.h"
#include <string>
#include <vector>
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"

#include "networktables\IntegerTopic.h"
#include "utils/logging/Logger.h"

using namespace std;

class DragonQuest : public DragonDataLogger

{
public:
    frc::Pose3d GetEstimatedPose();
    static DragonQuest *GetDragonQuest();
    void DataLog() override;

private:
    DragonQuest();
    ~DragonQuest() = default;
    bool IsConnected();
    double GetBatteryPercent();
    double GetTimeStamp();
    void ZeroHeading();
    void ZeroPosition();
    void CleanUpQuestMessages();
    units::angle::degree_t GetOculusYaw();
    void DoStuff();
    void ResetWithLimelightData();

    std::shared_ptr<nt::NetworkTable> m_networktable;
    std::shared_ptr<nt::NetworkTable> m_limelightNetworktable;
    static DragonQuest *m_dragonquest;
    double m_yawoffsetzero = 0;
    nt::IntegerSubscriber m_questMiso;
    nt::IntegerPublisher m_questMosi;
    nt::DoubleArrayTopic m_posTopic;
    nt::DoubleArrayTopic m_rotationTopic;
    nt::DoubleArrayTopic m_limelightPoseTopic;

    double m_xOffset = -0.254;
    double m_yOffset = 0;
    double m_zOffset = 0.47625;

    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 180;

    frc::Pose3d m_currentpos;
    double m_yaw = 0;

    int m_loopcounter = 0;
};