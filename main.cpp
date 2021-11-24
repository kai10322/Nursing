/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//#define B3_USE_STANDALONE_EXAMPLE 1

#define _DEBUG
// #define GL_FILL 0x1B02
// #define GL_FRONT_AND_BACK 0x0408

#ifdef _DEBUG
#pragma comment(lib, "BulletExampleBrowserLib_debug.lib")
#pragma comment(lib, "BulletCollision_debug.lib")
#pragma comment(lib, "BulletDynamics_debug.lib")
#pragma comment(lib, "BulletSoftBody_debug.lib")
#pragma comment(lib, "Bullet3Common_debug.lib")
#pragma comment(lib, "Bullet3Geometry_debug.lib")
#pragma comment(lib, "BulletFileLoader_debug.lib")
#pragma comment(lib, "BulletXmlWorldImporter_debug.lib")
#pragma comment(lib, "LinearMath_debug.lib")
#pragma comment(lib, "OpenGLWindow_debug.lib")
#pragma comment(lib, "BulletRobotics_debug.lib")
#else
#pragma comment(lib, "BulletExampleBrowserLib.lib")
#pragma comment(lib, "BulletCollision.lib")
#pragma comment(lib, "BulletDynamics.lib")
#pragma comment(lib, "BulletSoftBody.lib")
#pragma comment(lib, "Bullet3Common.lib")
#pragma comment(lib, "Bullet3Geometry.lib")
#pragma comment(lib, "BulletFileLoader.lib")
#pragma comment(lib, "BulletXmlWorldImporter.lib")
#pragma comment(lib, "LinearMath.lib")
#pragma comment(lib, "OpenGLWindow.lib")
#pragma comment(lib, "BulletRobotics.lib")
#endif
#pragma comment(lib, "opengl32.lib")

#include "Nursing.h"

#include <CommonInterfaces/CommonExampleInterface.h>
#include <CommonInterfaces/CommonGUIHelperInterface.h>

#include <Utils/b3Clock.h>

#include <OpenGLWindow/SimpleOpenGL3App.h>
#include <ExampleBrowser/OpenGLGuiHelper.h>
#include <ExampleBrowser/OpenGLExampleBrowser.h>

#include <LinearMath/btIDebugDraw.h>

#include <ThirdPartyLibs/glad/glad/gl.h>

#include <SharedMemory/SharedMemoryPublic.h>

CommonExampleInterface* example;
Nursing* nursing = 0;

int gSharedMemoryKey = -1;
static SharedMemoryInterface* sSharedMem = 0;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
  bool handled = false;
  handled = example->mouseMoveCallback(x, y);
  if (!handled)
    {
      if (prevMouseMoveCallback)
	prevMouseMoveCallback(x, y);
    }
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y)
{
  bool handled = false;
  
  handled = example->mouseButtonCallback(button, state, x, y);
  if (!handled)
    {
      if (prevMouseButtonCallback)
	prevMouseButtonCallback(button, state, x, y);
    }
}

b3KeyboardCallback prevKeyboardCallback = 0;
static bool renderVisualGeometry = true;
static bool gEnableDefaultKeyboardShortcuts = true;
static bool gEnableRenderLoop = true;
bool visualWireframe = false;
int gDebugDrawFlags = 0;

static void OnKeyboardCallback(int key, int state)
{
  // b3Printf("key=%d, state=%d\n", key, state);
  bool handled = false;

  handled = example->keyboardCallback(key, state);
  if (!handled)
  {
    if(prevKeyboardCallback)
      prevKeyboardCallback(key, state);
  }

  if(key == 'a' && state)
  {
    gDebugDrawFlags ^= btIDebugDraw::DBG_DrawAabb;
  }
  if(key == 'w' && state)
  {
    visualWireframe = !visualWireframe;
    gDebugDrawFlags ^= btIDebugDraw::DBG_DrawWireframe; 
    // if (renderVisualGeometry && ((gDebugDrawFlags & btIDebugDraw::DBG_DrawWireframe) == 0))
    if (gDebugDrawFlags & btIDebugDraw::DBG_DrawWireframe)
    {
      if (visualWireframe)
      {
        B3_PROFILE("physicsDebugDraw");
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        example->physicsDebugDraw(gDebugDrawFlags);
      }
      BT_PROFILE("Render Scene");
      example->renderScene();
      b3Printf("a\n");
    }
#if 1
    else
    {
      B3_PROFILE("physicsDebugDraw");
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      example->physicsDebugDraw(gDebugDrawFlags);
    }
#endif

    // b3Printf("Draw Wire Frame\n");
  }
  if(key == 'v' && state) 
  {
    renderVisualGeometry = !renderVisualGeometry;
  }
  if(key == 'f' && state){
    nursing->DrawContactForceFlag = !(nursing->DrawContactForceFlag);
  }
  if(key == 't' && state){
    nursing->DrawMotorForceFlag = !(nursing->DrawMotorForceFlag);
  }
}

class LessDummyGuiHelper : public DummyGUIHelper
{
  CommonGraphicsApp* m_app;
  
public:
  virtual CommonGraphicsApp* getAppInterface()
  {
    return m_app;
  }
  
  LessDummyGuiHelper(CommonGraphicsApp* app)
    : m_app(app)
  {
  }
};

void OpenGLExampleBrowserVisualizerFlagCallback(int flag, bool enable)
{
  if (flag == COV_ENABLE_RENDERING)
  {
    gEnableRenderLoop = (enable != 0);
  }

  if (flag == COV_ENABLE_KEYBOARD_SHORTCUTS)
  {
    gEnableDefaultKeyboardShortcuts = enable;
  }

  if (flag == COV_ENABLE_WIREFRAME)
  {
    visualWireframe = enable;
    if (visualWireframe)
    {
      gDebugDrawFlags |= btIDebugDraw::DBG_DrawWireframe;
    }
    else
    {
      gDebugDrawFlags &= ~btIDebugDraw::DBG_DrawWireframe;
    }
  }
}


int main(int argc, char* argv[])
{
  SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example", 1024, 768, true);
  
  prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
  prevMouseMoveCallback = app->m_window->getMouseMoveCallback();
  prevKeyboardCallback = app->m_window->getKeyboardCallback();
  
  app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
  app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
  app->m_window->setKeyboardCallback((b3KeyboardCallback)OnKeyboardCallback);
  
  OpenGLGuiHelper gui(app, false);

  gui.setVisualizerFlagCallback(OpenGLExampleBrowserVisualizerFlagCallback);
  //LessDummyGuiHelper gui(app);
  //DummyGUIHelper gui;
  
  CommonExampleOptions options(&gui);
  options.m_sharedMem = sSharedMem;
  
  //	example = StandaloneExampleCreateFunc(options);
  nursing = (Nursing*)NursingCreateFunc(options);
  example = nursing;
  example->processCommandLineArgs(argc, argv);
  
  example->initPhysics();
  example->resetCamera();
  
  b3Clock clock;
  
  do
    {
      app->m_instancingRenderer->init();
      app->m_instancingRenderer->updateCamera(app->getUpAxis());
      
      btScalar dtSec = btScalar(clock.getTimeInSeconds());
      if (dtSec > 0.1)
	dtSec = 0.1;
      
      example->stepSimulation(dtSec);
      clock.reset();

      example->renderScene();
      
      DrawGridData dg;
      dg.upAxis = app->getUpAxis();
      // app->drawGrid(dg);
      
      app->swapBuffer();
    } while (!app->m_window->requestedExit());
  
  example->exitPhysics();
  delete example;
  delete nursing;
  delete app;
  return 0;
}
