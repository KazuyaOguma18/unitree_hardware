/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#ifndef UNITREE_HARDWARE__VISIBILITY_CONTROL_H_
#define UNITREE_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UNITREE_HARDWARE_EXPORT __attribute__((dllexport))
#define UNITREE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define UNITREE_HARDWARE_EXPORT __declspec(dllexport)
#define UNITREE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef UNITREE_HARDWARE_BUILDING_DLL
#define UNITREE_HARDWARE_PUBLIC UNITREE_HARDWARE_EXPORT
#else
#define UNITREE_HARDWARE_PUBLIC UNITREE_HARDWARE_IMPORT
#endif
#define UNITREE_HARDWARE_PUBLIC_TYPE UNITREE_HARDWARE_PUBLIC
#define UNITREE_HARDWARE_LOCAL
#else
#define UNITREE_HARDWARE_EXPORT __attribute__((visibility("default")))
#define UNITREE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define UNITREE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define UNITREE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define UNITREE_HARDWARE_PUBLIC
#define UNITREE_HARDWARE_LOCAL
#endif
#define UNITREE_HARDWARE_PUBLIC_TYPE
#endif

#endif  // UNITREE_HARDWARE__VISIBILITY_CONTROL_H_