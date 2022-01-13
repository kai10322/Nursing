/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef _NURSING_H
#define _NURSING_H

#include "btBulletDynamicsCommon.h"
//#include "BulletSoftBody/btSoftBody.h"
// #include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
//#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
//#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
//#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

//#include "LinearMath/btAlignedObjectArray.h"
//#include "LinearMath/btQuickprof.h"
//#include "LinearMath/btIDebugDraw.h"

#include <ExampleBrowser/OpenGLGuiHelper.h>

// ---------------------------------------
// include files for create humanoid model
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "Utils/b3ResourcePath.h"
#include "Utils/b3BulletDefaultFileIO.h"
#include "CommonInterfaces/CommonMultiBodyBase.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h"

#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportMJCFDemo/BulletMJCFImporter.h"
// ---------------------------------------

#include <vector>

class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;


// #include "CommonInterfaces/CommonRigidBodyBase.h"

#define MAX_NUM_MOTORS 1024


struct ImportMJCFInternalData
{ 
  ImportMJCFInternalData()
	: m_numMotors(0),
	  m_mb(0),
          m_orientation(false)
  {
    for (int i = 0; i < MAX_NUM_MOTORS; i++)
    {
      m_jointMotors[i] = 0;
      m_generic6DofJointMotors[i] = 0; 
    }
  }

  btScalar m_motorTargetPositions[MAX_NUM_MOTORS];
  btMultiBodyJointMotor* m_jointMotors[MAX_NUM_MOTORS];
  btGeneric6DofSpring2Constraint* m_generic6DofJointMotors[MAX_NUM_MOTORS];
  int m_numMotors;
  btMultiBody* m_mb;
  // btAlignedObjectArray<btMultiBody*> m_humanoids;
  bool m_orientation;
  // int m_numHumanoids;
  // btRigidBody* m_rb;
};

class Nursing : public CommonMultiBodyBase
{
  char m_fileName[1024];

  struct ImportMJCFInternalData* m_data;
  btAlignedObjectArray<struct ImportMJCFInternalData*> m_datas;

  bool m_useMultiBody;
  btAlignedObjectArray<std::string*> m_nameMemory;
  btScalar m_grav;
  int m_upAxis;

public:
  btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

  btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

  btSoftBodyWorldInfo m_softBodyWorldInfo;

  bool m_autocam;
  bool m_cutting;
  bool m_raycast;
  btScalar m_animtime;
  btClock m_clock;
  int m_lastmousepos[2];
  btVector3 m_impact;
  btSoftBody::sRayCast m_results;
  btSoftBody::Node* m_node;
  btVector3 m_goal;
  bool m_drag;

  //keep the collision shapes, for deletion/cleanup
  btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

  btBroadphaseInterface* m_broadphase;

  btCollisionDispatcher* m_dispatcher;

  // btConstraintSolver* m_solver;
  btMultiBodyConstraintSolver* m_solver;

  btCollisionAlgorithmCreateFunc* m_boxBoxCF;

  btDefaultCollisionConfiguration* m_collisionConfiguration;

  bool DrawContactForceFlag = true;
  bool DrawMotorForceFlag = true;
  bool DrawSoftForceFlag = true;

  std::vector<std::string> vstr;
 
  int num_humanoid; // humanoidの数
  btScalar humanA_angle_array[23] = {0.f}; // jointの角度を格納
  btScalar humanB_angle_array[23] = {0.f}; // jointの角度を格納
  btScalar before_angle_array[23] = {0.f}; // jointの角度が計算できない時、直前の角度を使う 
  btScalar humanA_before_angle_array[23] = {0.f}; //  
  btScalar humanB_before_angle_array[23] = {0.f}; // 

public:
  void initPhysics();

  void exitPhysics();

  void stepSimulation(float deltaTime);

  void setFileName(const char* mjcfFileName);

  int createCheckeredTexture();

  void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld, btVector4 rgba = btVector4(1, 1, 1, 1));

  virtual void resetCamera()
  {
    //@todo depends on current_demo?
#if 0
    float dist = 45;
    float pitch = -28;
    float yaw = 136;
    float targetPos[3] = {2 - 1, 0};
#endif
#if 1	// MJCF Camera Setting
    float dist = 3.0;
    float pitch = -28;
    float yaw = -50;
    float targetPos[3] = {0.47, 0, -0.64};
#endif  
    m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
  }

#if 1
  Nursing(struct GUIHelperInterface* helper);
#endif

  virtual ~Nursing()
  {
    btAssert(m_dynamicsWorld == 0);

    // --------------------
    // delete humanoid data
    for (int i = 0; i < m_nameMemory.size(); i++)
    {
      delete m_nameMemory[i];
    }
    m_nameMemory.clear();
    delete m_data;
    // --------------------
  }

  //virtual void clientMoveAndDisplay();

  //virtual void displayCallback();

  void createStack(btCollisionShape* boxShape, float halfCubeSize, int size, float zPos);

  virtual void setDrawClusters(bool drawClusters);

  virtual const btSoftMultiBodyDynamicsWorld* getSoftDynamicsWorld() const
  {
    ///just make it a btSoftRigidDynamicsWorld please
    ///or we will add type checking
    return (btSoftMultiBodyDynamicsWorld*)m_dynamicsWorld;
  }

  virtual btSoftMultiBodyDynamicsWorld* getSoftDynamicsWorld()
  {
    ///just make it a btSoftRigidDynamicsWorld please
    ///or we will add type checking
    return (btSoftMultiBodyDynamicsWorld*)m_dynamicsWorld;
  }

  void mouseMotionFunc(int x, int y);

  GUIHelperInterface* getGUIHelper()
  {
    return m_guiHelper;
  }


  OpenGLGuiHelper* getOpenGLGuiHelper()
  {
    return (OpenGLGuiHelper*)m_guiHelper;
  }

  virtual void renderScene()
  {
    CommonMultiBodyBase::renderScene();
    btSoftMultiBodyDynamicsWorld* softWorld = getSoftDynamicsWorld();

    if(softWorld){
      // b3Printf("SoftBody.size() = %d\n",softWorld->getSoftBodyArray().size());
      for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
      // for(int i = 0; i < 0; i++)
      {
	btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
	//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
	{
	  // btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
	  // btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
	}
      }
    }
  }

  void DrawContactForce(btScalar fixedTimeStep = 1. / 60.f);
  void DrawMotorForce(btScalar fixedTimeStep = 1. / 60.f);
  void DrawSoftBodyAppliedForce(btScalar fixedTimeStep = 1. / 60.f);

  void judgeOrientation(struct ImportMJCFInternalData* m_data, const std::vector<std::string> &vstr, int n = 0);
  void readKeyPointFile(const char* filename, std::vector<std::string> &vstr);
  // void arrangeKeypointData(const char* output_filename);
  void arrangeKeypointData();
  void getPosData(const std::vector<std::string> &vstr, btScalar pos_data[][3], int frame = 0, int num = 0);
  void getInitRootJointPos(btScalar *root_joint_pos, int num = 0);
  void calcJointAngle(btScalar *angle_array, int frame = 0, int num = 0);
};

class CommonExampleInterface* NursingCreateFunc(struct CommonExampleOptions& options);

#endif  //_NURSING_H
