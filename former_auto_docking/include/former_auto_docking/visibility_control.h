#ifndef FORMER_AUTO_DOCKING__VISIBILITY_CONTROL_H_
#define FORMER_AUTO_DOCKING__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FORMER_AUTO_DOCKING_EXPORT __attribute__ ((dllexport))
    #define FORMER_AUTO_DOCKING_IMPORT __attribute__ ((dllimport))
  #else
    #define FORMER_AUTO_DOCKING_EXPORT __declspec(dllexport)
    #define FORMER_AUTO_DOCKING_IMPORT __declspec(dllimport)
  #endif
  #ifdef FORMER_AUTO_DOCKING_BUILDING_DLL
    #define FORMER_AUTO_DOCKING_PUBLIC FORMER_AUTO_DOCKING_EXPORT
  #else
    #define FORMER_AUTO_DOCKING_PUBLIC FORMER_AUTO_DOCKING_IMPORT
  #endif
  #define FORMER_AUTO_DOCKING_PUBLIC_TYPE FORMER_AUTO_DOCKING_PUBLIC
  #define FORMER_AUTO_DOCKING_LOCAL
#else
  #define FORMER_AUTO_DOCKING_EXPORT __attribute__ ((visibility("default")))
  #define FORMER_AUTO_DOCKING_IMPORT
  #if __GNUC__ >= 4
    #define FORMER_AUTO_DOCKING_PUBLIC __attribute__ ((visibility("default")))
    #define FORMER_AUTO_DOCKING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FORMER_AUTO_DOCKING_PUBLIC
    #define FORMER_AUTO_DOCKING_LOCAL
  #endif
  #define FORMER_AUTO_DOCKING_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // FORMER_AUTO_DOCKING__VISIBILITY_CONTROL_H_