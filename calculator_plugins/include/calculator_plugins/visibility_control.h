#ifndef CALCULATOR_PLUGINS__VISIBILITY_CONTROL_H_
#define CALCULATOR_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CALCULATOR_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define CALCULATOR_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define CALCULATOR_PLUGINS_EXPORT __declspec(dllexport)
    #define CALCULATOR_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CALCULATOR_PLUGINS_BUILDING_LIBRARY
    #define CALCULATOR_PLUGINS_PUBLIC CALCULATOR_PLUGINS_EXPORT
  #else
    #define CALCULATOR_PLUGINS_PUBLIC CALCULATOR_PLUGINS_IMPORT
  #endif
  #define CALCULATOR_PLUGINS_PUBLIC_TYPE CALCULATOR_PLUGINS_PUBLIC
  #define CALCULATOR_PLUGINS_LOCAL
#else
  #define CALCULATOR_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define CALCULATOR_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define CALCULATOR_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define CALCULATOR_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CALCULATOR_PLUGINS_PUBLIC
    #define CALCULATOR_PLUGINS_LOCAL
  #endif
  #define CALCULATOR_PLUGINS_PUBLIC_TYPE
#endif

#endif  // CALCULATOR_PLUGINS__VISIBILITY_CONTROL_H_
