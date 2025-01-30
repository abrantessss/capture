#ifndef CAPTURE_MSGS_CPP__VISIBILITY_CONTROL_H_
#define CAPTURE_MSGS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CAPTURE_MSGS_CPP_EXPORT __attribute__ ((dllexport))
    #define CAPTURE_MSGS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CAPTURE_MSGS_CPP_EXPORT __declspec(dllexport)
    #define CAPTURE_MSGS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CAPTURE_MSGS_CPP_BUILDING_DLL
    #define CAPTURE_MSGS_CPP_PUBLIC CAPTURE_MSGS_CPP_EXPORT
  #else
    #define CAPTURE_MSGS_CPP_PUBLIC CAPTURE_MSGS_CPP_IMPORT
  #endif
  #define CAPTURE_MSGS_CPP_PUBLIC_TYPE CAPTURE_MSGS_CPP_PUBLIC
  #define CAPTURE_MSGS_CPP_LOCAL
#else
  #define CAPTURE_MSGS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CAPTURE_MSGS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CAPTURE_MSGS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CAPTURE_MSGS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CAPTURE_MSGS_CPP_PUBLIC
    #define CAPTURE_MSGS_CPP_LOCAL
  #endif
  #define CAPTURE_MSGS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CAPTURE_MSGS_CPP__VISIBILITY_CONTROL_H_