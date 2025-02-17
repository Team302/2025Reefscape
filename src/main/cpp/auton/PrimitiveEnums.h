
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

#pragma once

// C++ Includes

// FRC includes

// Team 302 includes

// Third Party Includes

enum PRIMITIVE_IDENTIFIER
{
  UNKNOWN_PRIMITIVE = -1,
  DO_NOTHING,
  HOLD_POSITION,
  DRIVE_PATH_PLANNER,
  RESET_POSITION_PATH_PLANNER,
  VISION_ALIGN,
  DRIVE_TO_NOTE,
  DO_NOTHING_DELAY,
  DO_NOTHING_MECHANISMS,
  MAX_AUTON_PRIMITIVES
};
enum PATH_UPDATE_OPTION
{
  NOTHING = -1,
  RIGHT_REEF_BRANCH,
  LEFT_REEF_BRANCH,
  REEF_ALGAE,
  FLOOR_ALGAE,
  CORAL_STATION,
  PROCESSOR,
  MAX_DRIVE_TO_OPTIONS
};