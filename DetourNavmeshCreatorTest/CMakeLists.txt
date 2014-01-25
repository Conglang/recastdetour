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

SET(DetourNavmeshCreatorTest_SRCS
	Source/DetourChunkyTriMeshTest.cpp
	Source/DetourMeshTest.cpp
	Source/DetourNavmeshCfgTest.cpp
	Source/DetourNavmeshCreatorTest.cpp
	Source/Utils.cpp
	)
	
SET(DetourNavmeshCreatorTest_HDRS
	Include/Utils.h
	)

INCLUDE_DIRECTORIES(
	Include
	../Detour/Include
	../DetourNavmeshCreator/Include
	../DetourTileCache/Include
	../Recast/Include
	../DetourCrowdTest/Contrib/catch/include
	)
	
SOURCE_GROUP(sources FILES ${DetourNavmeshCreatorTest_SRCS} ${DetourNavmeshCreatorTest_HDRS})
SOURCE_GROUP(cmake FILES CMakeLists.txt)

ADD_EXECUTABLE(DetourNavmeshCreatorTest ${DetourNavmeshCreatorTest_SRCS} ${DetourNavmeshCreatorTest_HDRS})

SET_PROPERTY(TARGET DetourNavmeshCreatorTest PROPERTY DEBUG_POSTFIX -gd)
SET(DETOURNAVMESHCREATORTEST_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Bin)
SET_PROPERTY(TARGET DetourNavmeshCreatorTest PROPERTY RUNTIME_OUTPUT_DIRECTORY ${DETOURNAVMESHCREATORTEST_BIN_DIR})
IF(MSVC)
	# Enable some linker optimisations
	SET_PROPERTY(TARGET DetourNavmeshCreatorTest PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourNavmeshCreatorTest PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourNavmeshCreatorTest PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
ENDIF(MSVC)

TARGET_LINK_LIBRARIES(
	DetourNavmeshCreatorTest
	Detour
	DetourNavmeshCreator
	Recast
	RecastDetourDebugUtils
	)

ADD_TEST(
	NAME DetourMesh
	WORKING_DIRECTORY ${DETOURNAVMESHCREATORTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourNavmeshCreatorTest> [mesh])

ADD_TEST(
	NAME DetourChunkyTriMesh
	WORKING_DIRECTORY ${DETOURNAVMESHCREATORTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourNavmeshCreatorTest> [chunkyTriMesh])

ADD_TEST(
	NAME DetourNavmeshCfg
	WORKING_DIRECTORY ${DETOURNAVMESHCREATORTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourNavmeshCreatorTest> [navmeshCfg])

ADD_TEST(
	NAME DetourNavmeshCreator
	WORKING_DIRECTORY ${DETOURNAVMESHCREATORTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourNavmeshCreatorTest> [navmeshCreator])

