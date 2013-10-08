//
// Copyright (c) 2012 Clodéric Mars cloderic.mars@masagroup.net
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

#include "Visualization.h"

#include "InputGeom.h"
#include "SampleInterfaces.h"
#include "imgui.h"
#include "imguiRenderGL.h"
#include "DebugInfo.h"
#include "DetourSeekBehavior.h"
#include "DetourCollisionAvoidance.h"

#include <DetourCrowd.h>

#include <DetourCommon.h>
#include <DetourDebugDraw.h>

#include <Recast.h>
#include <RecastDebugDraw.h>

#include <SDL.h>
#include <SDL_opengl.h>

#include <cstdio>
#include <cfloat>

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) dtSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

Visualization::Visualization()
: m_zNear(1.f)
, m_zFar(1000.f)
, m_cameraPosition()
, m_cameraOrientation()
, m_cameraVelocity()
, m_scene(0)
, m_navmesh(0)
, m_crowd(0)
, m_debugInfo(0)
, m_crowdDt(0.05f)
, m_stopRequested(false)
, m_fullscreen(false)
, m_initialized(false)
, m_paused(false)
, m_winHeight()
, m_winWidth()
, m_lastTickCount()
, m_mousePosition()
, m_rotating(false)
, m_intialCameraOrientation()
, m_initialMousePosition()
, m_projection()
, m_modelView()
, m_viewport()
, m_crowdAvailableDt(0.f)
{
}

Visualization::~Visualization()
{
}

bool Visualization::isInitialized() const
{
    return m_initialized;
}

void Visualization::setFullscreen(bool fullscreen)
{
    m_fullscreen = fullscreen;
}

bool Visualization::isFullscreen() const
{
    return m_fullscreen;
}

bool Visualization::initialize()
{
    m_initialized = true;
    m_mousePosition[0] = m_mousePosition[1] = 1;
    m_crowdAvailableDt = 0.f;
    m_stopRequested = false;
	m_rotating = false;
	    
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialize SDL.\n");
		m_initialized = false;
	}
    else
    {
        // Center window
        char env[] = "SDL_VIDEO_CENTERED=1";
#ifdef _MSC_VER
        _putenv(env);
#else
        putenv(env);
#endif
        
        // Init OpenGL
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
        //#ifndef WIN32
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
        //#endif
        
        const SDL_VideoInfo* vi = SDL_GetVideoInfo();
        
        SDL_Surface* screen = 0;
        
        if (m_fullscreen)
        {
            m_winWidth = vi->current_w;
            m_winHeight = vi->current_h;
            screen = SDL_SetVideoMode(m_winWidth, m_winHeight, 0, SDL_OPENGL|SDL_FULLSCREEN);
        }
        else
        {	
            m_winWidth = vi->current_w - 20;
            m_winHeight = vi->current_h - 80;
            screen = SDL_SetVideoMode(m_winWidth, m_winHeight, 0, SDL_OPENGL);
        }
        
        if (!screen)
        {
            printf("Could not initialise SDL opengl.\n");
            m_initialized = false;
        }
        else
        {
            glEnable(GL_MULTISAMPLE);
            
            SDL_WM_SetCaption("Detour Crowd Demo", 0);
            
            if (!imguiRenderGLInit("DroidSans.ttf"))
            {
                printf("Could not init GUI renderer.\n");
                m_initialized = false;
            }
            else if (!initializeCamera())
            {
                printf("Unable to initialize the camera.\n");
                m_initialized = false;
            }
            else if (m_debugInfo == 0 || !m_debugInfo->initialize())
            {
                printf("Unable to initialize the crowd debug info.\n");
                m_initialized = false;
            }
            else
            {
                m_initialized = true;
                
                glEnable(GL_CULL_FACE);
                
                float fogCol[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
                glEnable(GL_FOG);
                glFogi(GL_FOG_MODE, GL_LINEAR);
                glFogf(GL_FOG_START, m_zFar*0.1f);
                glFogf(GL_FOG_END, m_zFar*1.25f);
                glFogfv(GL_FOG_COLOR, fogCol);
                
                glDepthFunc(GL_LEQUAL);
                
                m_lastTickCount = SDL_GetTicks();
                
                
                // Extract OpenGL view properties
                glGetDoublev(GL_PROJECTION_MATRIX, m_projection);
                glGetDoublev(GL_MODELVIEW_MATRIX, m_modelView);
                glGetIntegerv(GL_VIEWPORT, m_viewport);
            }
        }
    }
    
    if (!m_initialized)
    {
        SDL_Quit();
    }

    return m_initialized;
}

bool Visualization::terminate()
{
    m_debugInfo->terminate();
    
    imguiRenderGLDestroy();
	
	SDL_Quit();
    
    m_initialized = false;
    
    return !m_initialized;
}

bool Visualization::update()
{
    if (!m_initialized)
    {
        printf("Visualization has not been yet initialized.");
        return false;
    }
    
    // Compute the time since last update (in seconds).
    Uint32 time = SDL_GetTicks();
    float dt = (time - m_lastTickCount) / 1000.0f;
    m_lastTickCount = time;
    
    bool singleSimulationStep = false;
    bool scrollForward = false;
    bool scrollBackward = false;
    
    int mscroll = 0;
    
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
            case SDL_KEYDOWN:
                // Handle any key presses here.
                if (event.key.keysym.sym == SDLK_ESCAPE) //Escape.
                {
                    m_stopRequested = true;
                }
                else if (event.key.keysym.sym == SDLK_SPACE)
                {
                    m_paused = !m_paused;
                }
                else if (event.key.keysym.sym == SDLK_TAB)
                {
                    singleSimulationStep = true;
				}
				else if (event.key.keysym.sym == SDLK_r)
				{
					float pos[3] = {-15, 0, 15};
					m_crowd->pushAgentPosition(0, pos);
				}
				else if (event.key.keysym.sym == SDLK_KP7)
				{
					float pos[3] = {-19, 0, -19};
					dtVnormalize(pos);
					dtVscale(pos, pos, 2.f);

					dtCrowdAgent ag;
					m_crowd->fetchAgent(ag, 0);
					dtVcopy(ag.velocity, pos);
					m_crowd->pushAgent(ag);
				}
				else if (event.key.keysym.sym == SDLK_KP9)
				{
					float pos[] = {19, 0, -19};
					dtVnormalize(pos);
					dtVscale(pos, pos, 2.f);

					dtCrowdAgent ag;
					m_crowd->fetchAgent(ag, 0);
					dtVcopy(ag.velocity, pos);
					m_crowd->pushAgent(ag);
				}
				else if (event.key.keysym.sym == SDLK_KP3)
				{
					float pos[3] = {19, 0, 19};
					dtVnormalize(pos);
					dtVscale(pos, pos, 2.f);

					dtCrowdAgent ag;
					m_crowd->fetchAgent(ag, 0);
					dtVcopy(ag.velocity, pos);
					m_crowd->pushAgent(ag);
				}
				else if (event.key.keysym.sym == SDLK_KP1)
				{
					float pos[3] = {-19, 0, 19};
					dtVnormalize(pos);
					dtVscale(pos, pos, 2.f);

					dtCrowdAgent ag;
					m_crowd->fetchAgent(ag, 0);
					dtVcopy(ag.velocity, pos);
					m_crowd->pushAgent(ag);
				}
				else if (event.key.keysym.sym == SDLK_KP5)
				{
					float pos[3] = {0, 0, 0};
					dtVnormalize(pos);
					dtVscale(pos, pos, 2.f);

					dtCrowdAgent ag;
					m_crowd->fetchAgent(ag, 0);
					dtVcopy(ag.velocity, pos);
					m_crowd->pushAgent(ag);
				}
                break;
                
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    // Rotate view
                    m_rotating = true;
                    m_initialMousePosition[0] = m_mousePosition[0];
                    m_initialMousePosition[1] = m_mousePosition[1];
                    m_intialCameraOrientation[0] = m_cameraOrientation[0];
                    m_intialCameraOrientation[1] = m_cameraOrientation[1];
                    m_intialCameraOrientation[2] = m_cameraOrientation[2];
                }	
                else if (event.button.button == SDL_BUTTON_WHEELUP)
                {
                    scrollForward = true;
                }
				else if (event.button.button == SDL_BUTTON_WHEELDOWN)
				{
					scrollBackward = true;
				}
                break;
                
            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    m_rotating = false;
                }
                else if (event.button.button == SDL_BUTTON_LEFT)
                {
                    float pickedPosition[3];
					int index = 0;
                    pick(pickedPosition, &index);
                }
                break;
                
            case SDL_MOUSEMOTION:
                m_mousePosition[0] = event.motion.x;
                m_mousePosition[1] = m_winHeight-1 - event.motion.y;
                if (m_rotating)
                {
                    int dx = m_mousePosition[0] - m_initialMousePosition[0];
                    int dy = m_mousePosition[1] - m_initialMousePosition[1];
                    
                    m_cameraOrientation[0] = m_intialCameraOrientation[0] - dy*0.25f;
                    m_cameraOrientation[1] = m_intialCameraOrientation[1] + dx*0.25f;
                }
                break;
                
            case SDL_QUIT:
                m_stopRequested = true;
                break;
                
            default:
                break;
        }
    }
    
    unsigned char mbut = 0;
    if (SDL_GetMouseState(0,0) & SDL_BUTTON_LMASK)
        mbut |= IMGUI_MBUT_LEFT;
    if (SDL_GetMouseState(0,0) & SDL_BUTTON_RMASK)
        mbut |= IMGUI_MBUT_RIGHT;
    
    // Update the camera velocity from keyboard state.
    Uint8* keystate = SDL_GetKeyState(NULL);
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4800)
#endif
    updateCameraVelocity(
                         dt,
                         keystate[SDLK_w] || keystate[SDLK_UP] || scrollForward,
                         keystate[SDLK_s] || keystate[SDLK_DOWN] || scrollBackward,
                         keystate[SDLK_a] || keystate[SDLK_LEFT],
                         keystate[SDLK_d] || keystate[SDLK_RIGHT],
                         SDL_GetModState() & KMOD_SHIFT);
#ifdef _MSC_VER
#pragma warning(pop)
#endif

    //Update the camera position
    updateCameraPosition(dt);
    
    //Update the crowd
    if (m_crowd)
    {
        if (singleSimulationStep)
        {
            m_paused = true;
            m_debugInfo->startUpdate();

			m_crowd->update(m_crowdDt);

            m_debugInfo->endUpdate(m_crowdDt);
            m_crowdAvailableDt = 0.f;
        }
        else if (!m_paused)
        {
            m_crowdAvailableDt += dt;
            while(m_crowdAvailableDt > m_crowdDt)
            {
				m_debugInfo->startUpdate();

				m_crowd->update(m_crowdDt);

                m_debugInfo->endUpdate(m_crowdDt);
                m_crowdAvailableDt -= m_crowdDt;
            }
        }
        else
        {
            m_crowdAvailableDt = 0.f;
        }
    }
    
    // Set rendering context
    glViewport(0, 0, m_winWidth, m_winHeight);
    glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_TEXTURE_2D);
    
    // Render 3D
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0f, (float)m_winWidth/(float)m_winHeight, m_zNear, m_zFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(m_cameraOrientation[0],1,0,0);
    glRotatef(m_cameraOrientation[1],0,1,0);
    glRotatef(m_cameraOrientation[2],0,0,1);
    glTranslatef(-m_cameraPosition[0], -m_cameraPosition[1], -m_cameraPosition[2]);
    
    // Extract OpenGL view properties
    glGetDoublev(GL_PROJECTION_MATRIX, m_projection);
    glGetDoublev(GL_MODELVIEW_MATRIX, m_modelView);
    glGetIntegerv(GL_VIEWPORT, m_viewport);
    
    renderScene();
    renderCrowd();
    
    // Render 2D Overlay
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_winWidth, 0, m_winHeight);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
       
    imguiBeginFrame(m_mousePosition[0], m_mousePosition[1], mbut,mscroll);
    
    renderDebugInfoOverlay();
    
    imguiEndFrame();
    imguiRenderGLDraw();		
    
    glEnable(GL_DEPTH_TEST);
    SDL_GL_SwapBuffers();
    
    return true;
}

bool Visualization::pick(float* position, int* agentIdx)
{
    bool picked = false;
    
    if (m_scene)
    {
        float rays[3];
        float raye[3];
        float hitt;
        
        // Compute the ray
        GLdouble x, y, z;
        gluUnProject(m_mousePosition[0], m_mousePosition[1], 0.f, m_modelView, m_projection, m_viewport, &x, &y, &z);
        rays[0] = (float)x; rays[1] = (float)y; rays[2] = (float)z;
        gluUnProject(m_mousePosition[0], m_mousePosition[1], 1.f, m_modelView, m_projection, m_viewport, &x, &y, &z);
        raye[0] = (float)x; raye[1] = (float)y; raye[2] = (float)z;
        
        // ray cast
        if (m_scene->raycastMesh(rays, raye, hitt))
        {
            position[0] = rays[0] + (raye[0] - rays[0])*hitt;
            position[1] = rays[1] + (raye[1] - rays[1])*hitt;
            position[2] = rays[2] + (raye[2] - rays[2])*hitt;
            picked = true;
            if (m_crowd)
            {
                *agentIdx = -1;
                float tsel = FLT_MAX;
                float bmin[3], bmax[3];
                
                for (int i = 0, size(m_crowd->getAgentCount()); i < size; ++i)
                {
                    const dtCrowdAgent* ag = m_crowd->getAgent(i);
                    if (ag->active)
					{  
                        bmin[0] = ag->position[0] - ag->radius;
                        bmin[1] = ag->position[1];
                        bmin[2] = ag->position[2] - ag->radius;
                        bmax[0] = ag->position[0] + ag->radius;
                        bmax[1] = ag->position[1] + ag->height;
                        bmax[2] = ag->position[2] + ag->radius;
                        
                        float tmin, tmax;
                        if (isectSegAABB(rays, raye, bmin, bmax, tmin, tmax))
						{
                            if (tmin > 0 && tmin < tsel)
							{
								*agentIdx = i;
								tsel = tmin;
                                dtVcopy(position, ag->position);
                            } 
                        }
                    }
                }
            }

        }
    }
    return picked;
        
}

bool Visualization::initializeCamera()
{
    if (m_scene)
    {
        const float* max = m_scene->getMeshBoundsMax();
        const float* min = m_scene->getMeshBoundsMin();
        
        m_zFar = sqrtf(rcSqr(max[0]-min[0]) + rcSqr(max[1]-min[1]) + rcSqr(max[2]-min[2]));
        m_cameraPosition[0] = (max[0] + min[0]) / 2.f;
        m_cameraPosition[1] = (max[1] + min[1]) / 2.f + m_zFar;
        m_cameraPosition[2] = (max[2] + min[2]) / 2.f;
        m_zFar *= 2.f;
        
        m_cameraOrientation[0] = 90.f;
        m_cameraOrientation[1] = 0.f;
        m_cameraOrientation[2] = 0.f;
    }
    else
    {
        m_cameraPosition[0] = m_cameraPosition[1] = m_cameraPosition[2] = 0.f;
        m_cameraOrientation[0] = 45.f;
        m_cameraOrientation[1] = -45.f;
        m_cameraOrientation[2] = 0.f;
    }
    m_cameraVelocity[0] = m_cameraVelocity[1] = m_cameraVelocity[2] = 0.f;
    return true;
}

void Visualization::updateCameraVelocity(float dt, bool forward, bool backward, bool left, bool right, bool fast)
{
    float cameraKeySpeed = 22.0f;
    if (fast)
    {
        cameraKeySpeed *= 4.0f;
    }
    
    float cameraKeyAcceleration = 100.f;
    
    if (forward) //Forward
    {
        m_cameraVelocity[2] -= dt * cameraKeyAcceleration;
        m_cameraVelocity[2] = rcMax(m_cameraVelocity[2],-cameraKeySpeed);
    }
    else if (backward) // Backward
    {
        m_cameraVelocity[2] += dt * cameraKeyAcceleration;
        m_cameraVelocity[2] =  rcMin(m_cameraVelocity[2],cameraKeySpeed);
    }
    else if (m_cameraVelocity[2] > 0)
    {
        m_cameraVelocity[2] -= dt * cameraKeyAcceleration;
        m_cameraVelocity[2] =  rcMax(m_cameraVelocity[2],0.f);
    }
    else
    {
        m_cameraVelocity[2] += dt * cameraKeyAcceleration;
        m_cameraVelocity[2] =  rcMin(m_cameraVelocity[2],0.f);
    }
    
    if (right) // Right
    {
        m_cameraVelocity[0] += dt * cameraKeyAcceleration;
        m_cameraVelocity[0] =  rcMin(m_cameraVelocity[0],cameraKeySpeed);
    }
    else if (left) // Left
    {
        m_cameraVelocity[0] -= dt * cameraKeyAcceleration;
        m_cameraVelocity[0] = rcMax(m_cameraVelocity[0],-cameraKeySpeed);
    }
    else if (m_cameraVelocity[0] > 0)
    {
        m_cameraVelocity[0] -= dt * cameraKeyAcceleration;
        m_cameraVelocity[0] =  rcMax(m_cameraVelocity[0],0.f);
    }
    else
    {
        m_cameraVelocity[0] += dt * cameraKeyAcceleration;
        m_cameraVelocity[0] =  rcMin(m_cameraVelocity[0],0.f);
    }
}

void Visualization::updateCameraPosition(float dt)
{
    float cameraMove[3];
    for (unsigned int i(0) ; i < 3 ; ++i)
    {
        cameraMove[i] = m_cameraVelocity[i] * dt;
    }
    
    for (unsigned int i(0) ; i < 3 ; ++i)
    {
        for (unsigned int j(0) ; j < 3 ; ++j)
        {
            m_cameraPosition[j] += cameraMove[i] * (float)m_modelView[j*4 + i];
        }
    }
}

void Visualization::renderScene()
{
	if (m_scene)
    {
        DebugDrawGL dd;
        
        // Draw mesh
        duDebugDrawTriMesh(&dd, m_scene->getMesh()->getVerts(), m_scene->getMesh()->getVertCount(),
                           m_scene->getMesh()->getTris(), m_scene->getMesh()->getNormals(), m_scene->getMesh()->getTriCount(), 0, 1.0f);
        // Draw bounds
        const float* bmin = m_scene->getMeshBoundsMin();
        const float* bmax = m_scene->getMeshBoundsMax();
        duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
    }
}

void Visualization::renderCrowd()
{
    if (m_crowd)
    {
        DebugDrawGL dd;
        
        //Agents cylinders
        for (int i = 0, size(m_crowd->getAgentCount()); i < size; ++i)
        {
            const dtCrowdAgent* ag = m_crowd->getAgent(i);
            if (ag->active)
            {
                const float height = ag->height;
                const float radius = ag->radius;
                const float* pos = ag->position;
                
                unsigned int col = duRGBA(220,220,220,128);

				if (i >= 0 && i < 8) // Red
					col = duRGBA(220,0,0,128);
				else if (i >= 8 && i < 74) // Green
					col = duRGBA(0,220,0,128);
				else // Blue
					col = duRGBA(0,0,220,128);
                
                duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, duRGBA(0,0,0,32), 2.0f);
                duDebugDrawCircle(&dd, pos[0], pos[1]+height, pos[2], radius, duRGBA(0,0,0,32), 2.0f);
                duDebugDrawCylinder(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
                                    pos[0]+radius, pos[1]+height, pos[2]+radius, col);
            }
        }
        
        // Agents Trail
        for (int i = 0, size(m_crowd->getAgentCount()); i < size; ++i)
        {
            const dtCrowdAgent* ag = m_crowd->getAgent(i);
            if (ag->active)
            {
                const DebugInfo::AgentTrail* trail = &m_debugInfo->m_agentTrails[i];
                const float* pos = ag->position;
                
                dd.begin(DU_DRAW_LINES,3.0f);
                float prev[3];
                float preva = 1;
                dtVcopy(prev, pos);
                for (int j = 0; j < maxAgentTrailLen-1; ++j)
                {
                    const int idx = (trail->htrail + maxAgentTrailLen-j) % maxAgentTrailLen;
                    const float* v = &trail->trail[idx*3];
                    float a = 1 - j/(float)maxAgentTrailLen;
                    dd.vertex(prev[0],prev[1]+0.1f,prev[2], duRGBA(0,0,0,(int)(128*preva)));
                    dd.vertex(v[0],v[1]+0.1f,v[2], duRGBA(0,0,0,(int)(128*a)));
                    preva = a;
                    dtVcopy(prev, v);
                }
                dd.end();
            }
        }
        
        // Agents velocity
        for (int i = 0, size(m_crowd->getAgentCount()); i < size; ++i)
        {
            const dtCrowdAgent* ag = m_crowd->getAgent(i);
            if (ag->active)
            {
                const float radius = ag->radius;
                const float height = ag->height;
                const float* pos = ag->position;
                const float* vel = ag->velocity;
                const float* dvel = ag->desiredVelocity;
                
                unsigned int col = duRGBA(220,220,220,192);
				
				if (i >= 0 && i < 8) // Red
					col = duRGBA(220,0,0,192);
				else if (i >= 8 && i < 74) // Green
					col = duRGBA(0,220,0,192);
				else // Blue
					col = duRGBA(0,0,220,192);
                
                duDebugDrawCircle(&dd, pos[0], pos[1]+height, pos[2], radius, col, 2.0f);
                
				if (dtVlen(ag->desiredVelocity) > 0.1f)
				{
					duDebugDrawArrow(&dd, pos[0],pos[1]+height + 0.01f,pos[2],
						pos[0]+dvel[0],pos[1]+height+dvel[1] + 0.01f,pos[2]+dvel[2],
						0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
				}
                
                duDebugDrawArrow(&dd, pos[0],pos[1]+height + 0.01f,pos[2],
                                 pos[0]+vel[0],pos[1]+height+vel[1] + 0.01f,pos[2]+vel[2],
                                 0.0f, 0.4f, duRGBA(0,0,0,160), 2.0f);
            }
        }
       
        dd.depthMask(true);
    }
}

void Visualization::renderDebugInfoOverlay()
{
    GraphParams gp;
    gp.setRect(10, 10, 500, 200, 8);
    gp.setValueRange(0.0f, 2.0f, 4, "ms");
    
    drawGraphBackground(&gp);
    drawGraph(&gp, &m_debugInfo->m_crowdTotalTime, 1, "Crowd update", duRGBA(255,128,0,255));
    
    gp.setRect(10, 10, 500, 200, 8);
    gp.setValueRange(0.0f, 2000.0f, 1, "");
    drawGraph(&gp, &m_debugInfo->m_crowdSampleCount, 0, "Sample Count", duRGBA(96,96,96,128));
}
