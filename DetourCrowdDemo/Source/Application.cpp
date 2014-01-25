//
// Copyright (c) 2013 MASA Group recastdetour@masagroup.net
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Application.h"

Application::Application()
: m_context()
, m_sample()
, m_scene()
, m_navMesh()
, m_crowd()
, m_visu()
, m_debug()
{
	// NOTHING
}

Application::~Application()
{
	// NOTHING
}

bool Application::init(const char* fileName)
{
	if (!m_sample.loadFromFile(fileName, m_context))
		return false;

	m_debug.m_crowd = &m_sample.m_crowd;
	
	m_visu.m_scene = &m_sample.m_mesh;
	m_visu.m_crowd = &m_sample.m_crowd;
	m_visu.m_navmesh = &m_sample.m_navMesh;
	m_visu.m_debugInfo = &m_debug;

	//Run the simulation
	if (!m_visu.initialize())
		return false;

	return true;
}

bool Application::run()
{
	while (!m_visu.m_stopRequested)
	{
		if (!m_visu.update())
		{
			return false;
		}
	}
	
	if (!m_visu.terminate())
	{
		return false;
	}
	
	return false;
}
