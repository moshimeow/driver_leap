cmake_minimum_required(VERSION 3.12)
project(leap_driver_name VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)

# Make sure we have pretty colours
option(DISABLE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." OFF)

if(NOT DISABLE_COLORED_OUTPUT)
	if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		add_compile_options(-fdiagnostics-color=always)
	elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		add_compile_options(-fcolor-diagnostics)
	endif()
endif()


# Default to PIC code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MONADO_SRC_XRT ${CMAKE_CURRENT_SOURCE_DIR}/monado/src/xrt)

### Set configuration options for Monado

# Helper to obey CMP0077 and work around warnings.
function(setForce NAME VALUE)
	set(${NAME}
	    ${VALUE}
	    CACHE INTERNAL "" FORCE
		)
endfunction()

setforce(XRT_FEATURE_COMPOSITOR_MAIN OFF)

# No reason to disable this
setforce(XRT_FEATURE_COMPOSITOR_NULL ON)

# It looks like we can use Monado's normal out-of-proc OpenXR instance as-is. We'll see.
setforce(XRT_FEATURE_OPENXR ON)

setforce(XRT_FEATURE_RENDERDOC OFF) # we are not using this
setforce(XRT_FEATURE_STEAMVR_PLUGIN OFF) # we are not using this
setforce(XRT_FEATURE_TRACING OFF) # we are not using this yet
setforce(XRT_FEATURE_SLAM OFF) # we are not using this

setforce(XRT_FEATURE_CLIENT_DEBUG_GUI OFF) # we are not using this

setforce(XRT_MODULE_MONADO_CLI OFF) # we are not using this
setforce(XRT_MODULE_MONADO_GUI OFF) # we are not using this
setforce(XRT_MODULE_MERCURY_HANDTRACKING OFF)

setforce(XRT_HAVE_D3D11 OFF)
setforce(XRT_HAVE_D3D12 OFF)
setforce(XRT_HAVE_OPENCV OFF)

# No drivers!
setforce(XRT_BUILD_DRIVER_ANDROID OFF)
setforce(XRT_BUILD_DRIVER_ARDUINO OFF)
setforce(XRT_BUILD_DRIVER_DAYDREAM OFF)
setforce(XRT_BUILD_DRIVER_DEPTHAI OFF)
setforce(XRT_BUILD_DRIVER_EUROC OFF)
setforce(XRT_BUILD_DRIVER_HANDTRACKING OFF)
setforce(XRT_BUILD_DRIVER_TWRAP OFF)
setforce(XRT_BUILD_DRIVER_HDK OFF)
setforce(XRT_BUILD_DRIVER_HYDRA OFF)
setforce(XRT_BUILD_DRIVER_ILLIXR OFF)
setforce(XRT_BUILD_DRIVER_NS OFF)
setforce(XRT_BUILD_DRIVER_OHMD OFF)
setforce(XRT_BUILD_DRIVER_OPENGLOVES OFF)
setforce(XRT_BUILD_DRIVER_PSMV OFF)
setforce(XRT_BUILD_DRIVER_PSVR OFF)
setforce(XRT_BUILD_DRIVER_QWERTY OFF)
setforce(XRT_BUILD_DRIVER_REALSENSE OFF)
setforce(XRT_BUILD_DRIVER_REMOTE OFF)
setforce(XRT_BUILD_DRIVER_RIFT_S OFF)
setforce(XRT_BUILD_DRIVER_SURVIVE OFF)
setforce(XRT_BUILD_DRIVER_ULV2 OFF)
setforce(XRT_BUILD_DRIVER_VF OFF)
setforce(XRT_BUILD_DRIVER_VIVE OFF)
setforce(XRT_BUILD_DRIVER_WMR OFF)
setforce(XRT_BUILD_DRIVER_SIMULAVR OFF)

# There is no clean way to disable the prober/builders (as far as I can see?)
# So we enable one driver so that the legacy builder does not die
setforce(XRT_BUILD_DRIVER_SIMULATED ON)

setforce(XRT_BUILD_SAMPLES OFF)

setforce(BUILD_TESTING OFF)
setforce(BUILD_DOC OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# # Oh god
# set(leap_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}\\vendor\\leapSDK\\lib\\x64\\LeapC.lib")
# set(leap_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}\\vendor\\leapSDK\\include\\")
# find_package(leap REQUIRED)

add_library(leap SHARED IMPORTED)

set_target_properties(
    leap PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}\\vendor\\leapSDK\\include\\")
set_target_properties(
    leap
    PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES "C"
               IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}\\vendor\\leapSDK\\lib\\x64\\LeapC.dll"
               IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}\\vendor\\leapSDK\\lib\\x64\\LeapC.lib"
               )

# add_library(leap_header INTERFACE)
# target_include_directories(leap_header INTERFACE vendor/LeapSDK/include/)

# # target_link_directories(leap_header INTERFACE vendor/LeapSDK/lib/x64/)
# add_library(leap_binary SHARED IMPORTED vendor/LeapSDK/lib/x64/LeapC.dll)

add_subdirectory(vendor/monado)
add_subdirectory(vendor/glm)
add_subdirectory(vendor/pugixml)





add_library(make_includes_work INTERFACE)
target_include_directories(make_includes_work INTERFACE driver_leap/ driver_leap/Core driver_leap/Devices driver_leap/Utils)



add_library(driver_leap MODULE driver_leap/Core/CDriverConfig.cpp
    driver_leap/Core/CLeapPoller.cpp
    driver_leap/Core/CServerDriver.cpp

    driver_leap/Devices/CLeapController/CControllerButton.cpp
    driver_leap/Devices/CLeapController/CLeapController.cpp
    driver_leap/Devices/CLeapController/CLeapControllerIndex.cpp
    driver_leap/Devices/CLeapController/bones.cpp
    driver_leap/Devices/CLeapStation.cpp
    driver_leap/Utils/CGestureMatcher.cpp
    driver_leap/Utils/Utils.cpp
    driver_leap/dllmain.cpp
    driver_leap/stdafx.cpp
)


target_link_libraries(
        driver_leap
        PRIVATE
        make_includes_work
        leap
        xrt-external-openvr
        aux_util
        xrt-interfaces
        aux_math
        glm
        pugixml::pugixml
)
# copied from project_northstar_openvr_driver
target_compile_definitions(driver_leap PUBLIC -D_EXPORT)


# meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY driver.vrdrivermanifest DESTINATION ${PROJECT_BINARY_DIR}/leap)
file(COPY resources DESTINATION ${PROJECT_BINARY_DIR}/leap)


set(PLUGINDIR "${PROJECT_BINARY_DIR}/leap/bin/win64")

# message("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_leap PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

set_target_properties(driver_leap PROPERTIES PREFIX "")




install(
        DIRECTORY "${PROJECT_BINARY_DIR}/leap"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/share"
)
