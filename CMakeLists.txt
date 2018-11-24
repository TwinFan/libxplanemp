cmake_minimum_required(VERSION 3.0)

project(xplanemp LANGUAGES C CXX)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(XPSDK REQUIRED)


if(CMAKE_BUILD_TYPE MATCHES "Debug")
	set(XPMP_DEFINES ${XPMP_DEFINES} DEBUG=1)
	set(XPMP_DEBUG ON)
else()
	set(XPMP_DEBUG OFF)
endif()
# CMAKE_DEPENDENT_OPTION(XPMP_DEBUG_OPENGL "Install OpenGL Debug hooks and debug info" ON "XPMP_DEBUG" OFF)
if(XPMP_DEBUG_OPENGL)
	set(XPMP_DEFINES ${XPMP_DEFINES} DEBUG_GL=1)
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	set(XPMP_DEFINES ${XPMP_DEFINES} LIN=1)
	#set(XPMP_PLATFORM_SOURCES
	#	src/PlatformUtils.lin.cpp)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
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
	src/TexUtils.cpp
	src/XObjDefs.cpp
	src/XObjReadWrite.cpp
	src/XOGLUtils.cpp
	src/XPCAircraft.cpp
	src/XPMPMultiplayer.cpp
	src/XPMPMultiplayerCSL.cpp
	src/XPMPMultiplayerObj8.cpp
	src/XPMPMultiplayerObj.cpp
	src/XPMPMultiplayerVars.cpp
	src/XPMPPlaneRenderer.cpp
	src/XUtils.cpp)
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

message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")