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

SET(
	detourcrowdtest_SRCS
	Source/DetourBehaviorsTests.cpp
	Source/DetourCollisionAvoidanceTest.cpp
	Source/DetourCrowdTest.cpp
	Source/DetourOffMeshConnectionsTest.cpp
	Source/DetourPathFollowingTest.cpp
	Source/DetourPipelineTest.cpp
	Source/SinkingAgentRegressionTest.cpp
	Source/Utils.cpp
	)

SET(
	detourcrowdtest_HDRS
	Include/Utils.h
	)

SET(catch_INC Contrib/catch/include)

INCLUDE_DIRECTORIES(
	Include
	../Detour/Include
	../DetourCrowd/Include
	../DetourNavmeshCreator/Include
	../DetourTileCache/Include
	../Recast/Include
	${catch_INC}
	)

SOURCE_GROUP(sources FILES ${detourcrowdtest_SRCS} ${detourcrowdtest_HDRS})
SOURCE_GROUP(externals\\catch FILES ${catch_INC})
SOURCE_GROUP(cmake FILES CMakeLists.txt)

ADD_EXECUTABLE(DetourCrowdTest ${detourcrowdtest_SRCS} ${detourcrowdtest_HDRS})

SET_PROPERTY(TARGET DetourCrowdTest PROPERTY DEBUG_POSTFIX -gd)
SET(DETOURCROWDTEST_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Bin)
SET_PROPERTY(TARGET DetourCrowdTest PROPERTY RUNTIME_OUTPUT_DIRECTORY ${DETOURCROWDTEST_BIN_DIR})
IF(MSVC)
	# Enable some linker optimisations
	SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
ENDIF(MSVC)

TARGET_LINK_LIBRARIES(
	DetourCrowdTest
	Detour
	DetourCrowd
	DetourNavmeshCreator
	Recast
	RecastDetourDebugUtils
	)

ADD_TEST(
	NAME DetourCollisionAvoidance
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> [detourCollisionAvoidance])

ADD_TEST(
	NAME DetourCrowd
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> [detourCrowd])

ADD_TEST(
	NAME DetourBehavior
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> DetourBehaviorsTests/*)

ADD_TEST(
	NAME DetourPipelineBehavior
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> DetourPipelineTest/*)

ADD_TEST(
	NAME DetourPathFollowing
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> [detourPathFollowing])

ADD_TEST(
	NAME OffMeshConnections
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> [offmesh])

ADD_TEST(
	NAME Regression
	WORKING_DIRECTORY ${DETOURCROWDTEST_BIN_DIR}
	COMMAND $<TARGET_FILE:DetourCrowdTest> [regression])

