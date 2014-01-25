*currently [![Build status](https://travis-ci.org/masagroup/recastdetour.png?branch=master)](https://travis-ci.org/masagroup/recastdetour)*

*contributions are welcome, using pull requests to the [integration](https://github.com/masagroup/recastdetour/tree/integration) branch*

# About #

This repository is a fork of [recastnavigation](https://github.com/memononen/recastnavigation) whose original author is [Mikko Mononen](memon@inside.org).
The fork is maintainted by MASA Group´s product team of [MASA LIFE](http://masalife.net).
To see our latest news in artificial intelligence you can follow our tweet - [@masalife_ai](https://twitter.com/masalife_ai).

[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/memononen/recastnavigation/trend.png)](https://bitdeli.com/free "Bitdeli Badge")

This library is releaed under the terms of the open souce [Zlib license](http://opensource.org/licenses/Zlib).
![screenshot of a navmesh baked with the sample program](/RecastDemo/screenshot.png?raw=true)

## Recast ##

Recast is state of the art navigation mesh construction toolset for games.

* It is automatic, which means that you can throw any level geometry at it and you will get robust mesh out
* It is fast which means swift turnaround times for level designers
* It is open source so it comes with full source and you can customize it to your hearts content. 

The Recast process starts with constructing a voxel mold from a level geometry 
and then casting a navigation mesh over it. The process consists of three steps, 
building the voxel mold, partitioning the mold into simple regions, peeling off 
the regions as simple polygons.

1. The voxel mold is build from the input triangle mesh by rasterizing the triangles into a multi-layer heightfield. Some simple filters are  then applied to the mold to prune out locations where the character would not be able to move.
2. The walkable areas described by the mold are divided into simple overlayed 2D regions. The resulting regions have only one non-overlapping contour, which simplifies the final step of the process tremendously.
3. The navigation polygons are peeled off from the regions by first tracing the boundaries and then simplifying them. The resulting polygons are finally converted to convex polygons which makes them perfect for pathfinding and spatial reasoning about the level. 


## Detour ##

Recast is accompanied with Detour, path-finding and spatial reasoning toolkit. You can use any navigation mesh with Detour, but of course the data generated with Recast fits perfectly.

Detour offers simple static navigation mesh which is suitable for many simple cases, as well as tiled navigation mesh which allows you to plug in and out pieces of the mesh. The tiled mesh allows to create systems where you stream new navigation data in and out as the player progresses the level, or you may regenerate tiles as the world changes. 

## Detour Crowd ##

The Detour Crowd module provides you with features for agents handling and behavior customization. You can create lots of agents and move them in the navigation mesh thanks to Detour. Moreover you can create your own behaviors that will tell your agents how to move and to react.

# Platforms/Compatibility #

**Recast/Detour** is written in [C++03](http://en.wikipedia.org/wiki/C%2B%2B03). It has been tested on several platforms:

- Mac OS X Mountain Lion (x64),
  - clang 4.2 (clang-425.0.24);
- Windows 7 (x86 & x64),
  - Visual Studio 2010 SP1 (10.0.40219.1 SP1Rel);
- Debian Linux 6.05 (x86),
  - GCC 4.4.5;
- Ubuntu Linux 12.04 (x86) (using [Travis](https://travis-ci.org/masagroup/recastdetour)),
  - GCC 4.6.x,
  - Clang 3.1.x.

# Build instructions #

## Windows/Visual Studio ##

### Prerequisites ###
- [Cmake](http://www.cmake.org/);
- [SDL](http://www.libsdl.org) development libraries;
- [Catch](https://github.com/philsquared/Catch) unit test library is used as a [submodule](http://git-scm.com/book/en/Git-Tools-Submodules) in [DetourCrowdTest/Contrib/catch](DetourCrowdTest/Contrib/catch);
- Virtually any recent version of visual studio, tested with,
    - Visual Studio 2010 x86,
    - Visual Studio 2010 x64.

### Build ###
1. Generate the Visual Studio files with CMake.
    - Set the source code directory to the root of the repository (e.g. `E:\recastdetour`);
    - Set the Cmake build directory to where you desire the `.sln`, `.vcproj` and co. to be (e.g. `E:\recastdetour\Build\vc100`);
    - Set the `SDL_INCLUDE_DIR` to the path of `include` directory part of the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/include`);
    - Set the `SDL_LIBRARY_TEMP` to the path of `SDL.lib` downloaded with the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/lib/x86/SDL.lib`);
    - Set the `SDLMAIN_LIBRARY` to the path of `SDLMain.lib` downloaded with the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/lib/x86/SDLmain.lib`);
    - Click *Configure* then *Generate*.
2. Initialize and update the submodule (git submodule init & git submodule update)
3. Build with Visual Studio.
    - Open `RecastNavigation.sln` that has been generated in the chosen directory (e.g. `E:\recastdetour\Build\RecastNavigation.sln`);
    - Build the project `ALL_BUILD`, it will build all libraries and executables.
    - Build the project `RUN_TESTS`, it will launch the unit tests and check that they passed successfully.
4. Execute `Recast_Demo` and `DetourCrowd_Demo`.
    - Make sure you execute the application from its Run directory (e.g. `E:\projects\recastdetour\RecastDemo\Run`);
    - Make sure `SDL.dll` is present in the path;
    - Run, and enjoy!

# Support #

- A Google Group is available [here](https://groups.google.com/forum/?fromgroups#!forum/recastnavigation)
- The online documentation of the latest stable version is available at [masagroup.github.io/recastdetour](http://masagroup.github.io/recastdetour/)
- If you notice a bug or something strange, please let us know about it, create an [issue](https://github.com/masagroup/recastdetour/issues)

# Contributors #

## [Origin](https://github.com/memononen/recastnavigation) repository contributors ##
- Mikko Mononen <memon@inside.org>
- Stephen Pratt <stevefsp@gmail.com>
- Cameron Hart <cameron.hart@gmail.com>

## [MASA LIFE](http://www.masalife.net) team contributors ##
- Clodéric Mars <cloderic.mars@masagroup.net>
- Damien Avrillon <damien.avrillon@masagroup.net>
- Joel Joly <joel.joly@masagroup.net>

## Other contributors ##
- Jérémy Chanut <jeremy.chanut@gmail.com>
- Charles Prévot <prevot@cervval.com>

# Release Notes #

## Recast/Detour 2.0.2 ##
**Released October 8th, 2013**

- Overhaul of `dtCollisionAvoidance`: parameters moved to the behavior itself and better documentation.
- Overhaul of `dtPathFollowing`: parameters moved to the behavior itself, target setting moved to the params and better documentation.
- Clearer lifetime for agents desired velocity.
- Renaming of dtVperp2D tp dtVCross2D for better consistency.
- Renaming `dtCrowd::setAgentBehavior` to `dtCrowd::pushAgentBehavior`.
- Renaming `dtCrowd::updateAgentPosition` to `dtCrowd::pushAgentPosition`.
- Renaming `dtCrowd::applyAgent` to `dtCrowd::pushAgent`.
- Addition of the *mazy hall* DetourCrowd sample.
- Additional DetourCrowd unit tests.
- Attaching the revision tree to the new [*origin* github repository](https://github.com/memononen/recastnavigation).
- Fixing the build of DetourCrowdDemo on linux.

## Recast/Detour 2.0.1 ##
**Released August 22nd, 2013**

- Removing the proximity grid (doesn't handle overlapping agents and 3D levels).
- Bug fixing on path following.
- Features to control the offMesh connections.
- Path Following parameters no longer have to be initialized manually.

## Recast/Detour 2.0 ##
**Released July 15th, 2013**

- The interface for the dtCrowd class has been completely rebuild.
- Implementation of the notion of behaviors for the agents.
- Some behaviors are available by default for the user to use.
- The user can create its own behaviors.
- The documentation has been updated and some tutorials have been added.
- The features of the Detour Crowd module have unit tests (more than 100)

## Recast 1.4 ##
**Released August 24th, 2009**

- Added detail height mesh generation (RecastDetailMesh.cpp) for single,
  tiled statmeshes as well as tilemesh.
- Added feature to contour tracing which detects extra vertices along
  tile edges which should be removed later.
- Changed the tiled stat mesh preprocess, so that it first generated
  polymeshes per tile and finally combines them.
- Fixed bug in the GUI code where invisible buttons could be pressed.

## Recast 1.31 ##
**Released July 24th, 2009**

- Better cost and heuristic functions.
- Fixed tile navmesh raycast on tile borders.

## Recast 1.3 ##
**Released July 14th, 2009**

- Added dtTileNavMesh which allows to dynamically add and remove navmesh pieces at runtime.
- Renamed stat navmesh types to dtStat* (i.e. dtPoly is now dtStatPoly).
- Moved common code used by tile and stat navmesh to DetourNode.h/cpp and DetourCommon.h/cpp.
- Refactores the demo code.

## Recast 1.2 ##
**Released June 17th, 2009**
- *Linux*: `premake4 gmake`

- Added tiled mesh generation. The tiled generation allows to generate navigation for
  much larger worlds, it removes some of the artifacts that comes from distance fields
  in open areas, and allows later streaming and dynamic runtime generation
- Improved and added some debug draw modes
- API change: The helper function rcBuildNavMesh does not exists anymore,
  had to change few internal things to cope with the tiled processing,
  similar API functionality will be added later once the tiled process matures
- The demo is getting way too complicated, need to split demos
- Fixed several filtering functions so that the mesh is tighter to the geometry,
  sometimes there could be up error up to tow voxel units close to walls,
  now it should be just one.

## Recast 1.1 ##
**Released April 11th, 2009**

This is the first release of Detour.

## Recast 1.0 ##
**Released March 29th, 2009**

This is the first release of Recast.

The process is not always as robust as I would wish. The watershed phase sometimes swallows tiny islands
which are close to edges. These droppings are handled in rcBuildContours, but the code is not
particularly robust either.

Another non-robust case is when portal contours (contours shared between two regions) are always
assumed to be straight. That can lead to overlapping contours specially when the level has
large open areas.


[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/masagroup/recastdetour/trend.png)](https://bitdeli.com/free "Bitdeli Badge")

Recast & Detour is licensed under ZLib license, see License.txt for more information.
