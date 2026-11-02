#pragma once

/**
 * Shared-library export/import macros for Windows DLL builds.
 * On Linux / macOS symbols are visible by default.
 */
#if defined(_WIN32) || defined(__CYGWIN__)
  #if defined(MOTOR_CONTROLLER_SHARED)
    #if defined(MOTOR_CONTROLLER_BUILDING)
      #define MOTOR_CONTROLLER_API __declspec(dllexport)
    #else
      #define MOTOR_CONTROLLER_API __declspec(dllimport)
    #endif
  #else
    #define MOTOR_CONTROLLER_API
  #endif
#else
  #define MOTOR_CONTROLLER_API
#endif
