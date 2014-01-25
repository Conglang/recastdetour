#
# Copyright (c) 2013 MASA Group recastdetour@masagroup.net
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
#    claim that you wrote the original software. If you use this software
#    in a product, an acknowledgment in the product documentation would be
#    appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
#    misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(DetourNavmeshCreator_SRCS
	Source/DetourChunkyTriMesh.cpp
	Source/DetourMesh.cpp
	Source/DetourNavmeshCfg.cpp
	Source/DetourNavmeshCreator.cpp
)

SET(DetourNavmeshCreator_HDRS
	Include/DetourChunkyTriMesh.h
	Include/DetourMesh.h
	Include/DetourNavmeshCfg.h
	Include/DetourNavmeshCreator.h
)

SOURCE_GROUP(sources FILES ${DetourNavmeshCreator_SRCS} ${DetourNavmeshCreator_HDRS})
SOURCE_GROUP(cmake FILES CMakeLists.txt)

INCLUDE_DIRECTORIES(
	Include 
	${simplejson_DIR}
	../RecastDetourDebugUtils/Include
	../Detour/Include
	../DetourCrowd/Include
	../DetourTileCache/Include
	../Recast/Include
)

ADD_LIBRARY(DetourNavmeshCreator ${DetourNavmeshCreator_SRCS} ${DetourNavmeshCreator_HDRS})
IF(IOS)
	# workaround a bug forbidding to install the built library (cf. http://www.cmake.org/Bug/view.php?id=12506)
	SET_TARGET_PROPERTIES(DetourNavmeshCreator PROPERTIES 
		ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug"
		ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
ENDIF()

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "Building binaries containing both 32 and 64 bits versions." 
	ON "APPLE;NOT IOS" 
	OFF)
IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
	SET_TARGET_PROPERTIES(DetourNavmeshCreator PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
ENDIF()

# To fix compilation problem: relocation R_X86_64_32 against `a local symbol' can not be
# used when making a shared object; recompile with -fPIC
# See http://www.cmake.org/pipermail/cmake/2007-May/014350.html
IF( UNIX )
	SET_TARGET_PROPERTIES(DetourNavmeshCreator PROPERTIES COMPILE_FLAGS "-fPIC")
ENDIF()

INSTALL(FILES ${DetourNavmeshCreator_HDRS}
	DESTINATION include/detourNavmeshCreator
)

INSTALL(TARGETS DetourNavmeshCreator EXPORT "RecastDetourTargets-${PLATFORM_STR}"
	RUNTIME DESTINATION bin/${PLATFORM_STR}
	LIBRARY DESTINATION lib/${PLATFORM_STR}
	ARCHIVE DESTINATION lib/${PLATFORM_STR}
)

TARGET_LINK_LIBRARIES(DetourNavmeshCreator Detour DetourCrowd)

if(WIN32)
	set(CMAKE_CXX_FLAGS "/EHsc /GR- /W4")
else()
	set(CMAKE_CXX_FLAGS "-Wall -ansi -O3")
endif()
