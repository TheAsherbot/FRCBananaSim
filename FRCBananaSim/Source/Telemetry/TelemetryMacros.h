/**
 * @file TelemetryMacros.h
 * @brief Preprocessor macros for zero-overhead performance instrumentation across build configurations.
 */

#pragma once

#include "Telemetry/TelemetryTimer.h"

#if defined(_DEBUG) || defined(SIM_FORCE_PROFILING)
#define SIM_ENABLE_PROFILING 1
#else
#define SIM_ENABLE_PROFILING 0
#endif

#if SIM_ENABLE_PROFILING
 /**
  * @def SIM_PROFILE_SCOPE(name)
  * @brief Measures execution duration for the enclosing code block under a custom label.
  */
#define SIM_PROFILE_SCOPE(name) ::FRCBananaSim::Telemetry::TelemetryTimer timer##__LINE__(name)

  /**
   * @def SIM_PROFILE_FUNCTION()
   * @brief Measures execution duration for the enclosing function using compiler function metadata.
   */
#define SIM_PROFILE_FUNCTION() SIM_PROFILE_SCOPE(__FUNCTION__)
#else
#define SIM_PROFILE_SCOPE(name)
#define SIM_PROFILE_FUNCTION()
#endif