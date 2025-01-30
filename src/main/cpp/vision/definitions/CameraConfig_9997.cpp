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

#include "vision/definitions/CameraConfig_9997.h"
#include "vision/DragonVision.h"
#include "vision/DragonLimelight.h"
#include "utils/logging/Logger.h"

void CameraConfig_9997::DefineCameras()
{

    DragonLimelight *placer = new DragonLimelight(std::string("limelight-placer"),                   // networkTableName
                                                  DragonCamera::CAMERA_TYPE::LIMELIGHT3,             // PIPELINE initialPipeline,
                                                  DragonCamera::CAMERA_USAGE::GAME_ELEMENT_DETECTION,             // PIPELINE initialPipeline,
                                                  units::length::inch_t(0),                          // units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
                                                  units::length::inch_t(0),                          // units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
                                                  units::length::inch_t(0),                          // units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
                                                  units::angle::degree_t(0),                         // units::angle::degree_t pitch,          /// <I> - Pitch of camera
                                                  units::angle::degree_t(0),                         // units::angle::degree_t yaw,            /// <I> - Yaw of camera
                                                  units::angle::degree_t(0),                         // units::angle::degree_t roll,           /// <I> - Roll of camera
                                                  DragonLimelight::LL_PIPELINE::MACHINE_LEARNING_PL, /// <I> enum for starting pipeline
                                                  DragonLimelight::LED_MODE::LED_OFF,                // LED_MODE ledMode,
                                                  DragonLimelight::CAM_MODE::CAM_VISION,             // CAM_MODE camMode,
                                                  DragonLimelight::STREAM_MODE::STREAM_STANDARD,      // STREAM_MODE streamMode,
                                                  DragonLimelight::SNAPSHOT_MODE::SNAP_OFF           // SNAPSHOT_MODE snapMode
    );                                                                                               // additional parameter
    DragonVision::GetDragonVision()->AddCamera(placer, RobotElementNames::CAMERA_USAGE::PLACER);
}
