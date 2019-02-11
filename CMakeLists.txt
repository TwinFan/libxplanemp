cmake_minimum_required(VERSION 3.0)

project(xplanemp LANGUAGES C CXX)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(XPSDK REQUIRED)

# Define my plugin's name
set(XPMP_DEFINES ${XPMP_DEFINES} XPMP_CLIENT_NAME="LT" XPMP_CLIENT_LONGNAME="LiveTraffic")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
	set(XPMP_DEFINES ${XPMP_DEFINES} DEBUG=1)
	set(XPMP_DEBUG ON)
else()
	set(XPMP_DEBUG OFF)
endif()
option(XPMP_DEBUG_OPENGL "Install OpenGL Debug hooks and debug info" OFF)
if(XPMP_DEBUG_OPENGL)
	set(XPMP_DEFINES ${XPMP_DEFINES} DEBUG_GL=1)
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    # Always use position-independent code
    add_compile_options(-O3 -fPIC)
    if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
        # highest optimization level in Release builds
        add_compile_options(-O3 -fPIC)
	endif()
    set(XPMP_DEFINES ${XPMP_DEFINES} LIN=1)
	#set(XPMP_PLATFORM_SOURCES
	#	src/PlatformUtils.lin.cpp)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
        # Use highest optimization level in Release builds
        add_compile_options(/GL)
    endif()
	set(XPMP_DEFINES ${XPMP_DEFINES} IBM=1 _CRT_SECURE_NO_WARNINGS)
	#set(XPMP_PLATFORM_SOURCES
	#	src/PlatformUtils.win.cpp)
	set(XPMP_PLATFORM_LIBRARIES opengl32 glu32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
	set(XPMP_DEFINES ${XPMP_DEFINES} APL=1)
	#set(XPMP_PLATFORM_SOURCES
	#	src/PlatformUtils.mac.cpp)
	find_library(OPENGL OpenGL)
	set(XPMP_PLATFORM_LIBRARIES ${OPENGL})
endif()
#if(PNG_FOUND)
	add_definitions(-DBITMAP_USE_PNG=1)
#endif()

add_definitions(-DXPLM200=1 -DXPLM210=1)

add_library(xplanemp
	${XPMP_PLATFORM_SOURCES}
	src/BitmapUtils.cpp
	src/BitmapUtils.h
	src/TexUtils.cpp
	src/TexUtils.h
	src/XObjDefs.cpp
	src/XObjDefs.h
	src/XObjReadWrite.cpp
	src/XObjReadWrite.h
	src/XOGLUtils.cpp
	src/XOGLUtils.h
	src/XPCAircraft.cpp
	include/XPCAircraft.h
	src/XPMPMultiplayer.cpp
	include/XPMPMultiplayer.h
	src/XPMPMultiplayerCSL.cpp
	src/XPMPMultiplayerCSLOffset.cpp
	src/XPMPMultiplayerObj8.cpp
	src/XPMPMultiplayerObj8.h
	src/XPMPMultiplayerObj.cpp
	src/XPMPMultiplayerObj.h
	src/XPMPMultiplayerVars.cpp
	src/XPMPMultiplayerVars.h
	src/XPMPPlaneRenderer.cpp
	include/XPMPPlaneRenderer.h
	src/XUtils.cpp
	src/XUtils.h
	src/XStringUtils.h
	src/ResourceManager.h
	src/Interpolation.h)
target_include_directories(xplanemp
	PUBLIC
		${ADDITIONAL_INCLUDES}
		${XPSDK_INCLUDE_DIRS}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(xplanemp
	PRIVATE ${XPSDK_XPLM_LIBRARIES}
	${PNG_LIBRARY}
	${XPMP_PLATFORM_LIBRARIES})
target_compile_definitions(xplanemp PRIVATE ${XPMP_DEFINES} PUBLIC XUTILS_EXCLUDE_MAC_CRAP=1)
set_property(TARGET xplanemp PROPERTY CXX_STANDARD_REQUIRED 11)
set_property(TARGET xplanemp PROPERTY CXX_STANDARD 14)
