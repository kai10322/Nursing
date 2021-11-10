/*
  Bullet Continuous Collision Detection and Physics Library
  Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

  This software is provided 'as-is', without any express or implied warranty.
  In no event will the authors be held liable for any damages arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose, 
  including commercial applications, and to alter it and redistribute it freely, 
  subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h" //

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "BunnyMesh.h"
#include <stdio.h>  //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "Nursing.h"
#include "ExampleBrowser/GL_ShapeDrawer.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody.h"

#include "CommonInterfaces/CommonParameterInterface.h"
#include "CommonInterfaces/CommonGraphicsAppInterface.h"

// ---------------------------------------
// using include files
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
// ---------------------------------------

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftMultiBodynamicsWorld;

#include "CommonInterfaces/CommonMultiBodyBase.h"

#include "BedFrame.h"
#include "Mattress.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

//static bool sDemoMode = false;

const int maxProxies = 32766;
//const int maxOverlap = 65535;

static btVector3* gGroundVertices = 0;
static int* gGroundIndices = 0;
//static btBvhTriangleMeshShape* trimeshShape =0;
//static btRigidBody* staticBody = 0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE = 8.f;
int current_demo = 20;
#define DEMO_MODE_TIMEOUT 15.f  //15 seconds for each demo

#ifdef _DEBUG
//const int gNumObjects = 1;
#else
//const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

//const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f
#define MAX_NUM_MOTORS 1024

//

// ---------------------
// variable to store mjcf file names
static btAlignedObjectArray<std::string> gMCFJFileNameArray;
// --------------------

// ----------------------------------------------------------------------------------
// constructor 
Nursing::Nursing(struct GUIHelperInterface* helper, int option, const char* fileName)
    : CommonMultiBodyBase(helper),
      m_drag(false)
{
  m_data = new ImportMJCFInternalData;

  m_useMultiBody = true;

  static int count = 0;
  if (fileName)
  {
    setFileName(fileName);
  }
  else
  {
    gMCFJFileNameArray.clear();

    //load additional MJCF file names from file

    FILE* f = fopen("mjcf_files.txt", "r");
    if (f)
    {
      int result;
      //warning: we don't avoid string buffer overflow in this basic example in fscanf
      char fileName[1024];
      do
      {
	result = fscanf(f, "%s", fileName);
	b3Printf("mjcf_files.txt entry %s", fileName);
	if (result == 1)
	{
	  gMCFJFileNameArray.push_back(fileName);
	}
       } while (result == 1);

       fclose(f);
    }

    if (gMCFJFileNameArray.size() == 0)
    {
	  gMCFJFileNameArray.push_back("./humanoid.xml");
    }
    int numFileNames = gMCFJFileNameArray.size();

    if (count >= numFileNames)
    {
	    count = 0;
    }
    sprintf(m_fileName, "%s", gMCFJFileNameArray[count++].c_str());
  }

  for(int i = 0; i < gMCFJFileNameArray.size(); i++){
    b3Printf("gMCFJFileNameArray[%d] = %s\n", i, gMCFJFileNameArray[i].c_str());
  }
}
// ----------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
// set file name
void Nursing::setFileName(const char* mjcfFileName)
{
  memcpy(m_fileName, mjcfFileName, strlen(mjcfFileName) + 1);
}

// ----------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
// error logger
struct MyMJCFLogger : public MJCFErrorLogger
{
  virtual void reportError(const char* error)
  {
	  b3Error(error);
  }
  virtual void reportWarning(const char* warning)
  {
	  b3Warning(warning);
  }
  virtual void printMessage(const char* msg)
  {
	  b3Printf(msg);
  }
};
// ----------------------------------------------------------------------------------

void Nursing::createStack(btCollisionShape* boxShape, float halfCubeSize, int size, float zPos)
{
  btTransform trans;
  trans.setIdentity();

  for (int i = 0; i < size; i++)
    {
      // This constructs a row, from left to right
      int rowSize = size - i;
      for (int j = 0; j < rowSize; j++)
	{
	  btVector3 pos;
	  pos.setValue(
		       -rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
		       halfCubeSize + i * halfCubeSize * 2.0f,
		       zPos);

	  trans.setOrigin(pos);
	  btScalar mass = 1.f;

	  btRigidBody* body = 0;
	  body = createRigidBody(mass, trans, boxShape);
	}
    }
}


////////////////////////////////////
///for mouse picking
void pickingPreTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
  Nursing* softDemo = (Nursing*)world->getWorldUserInfo();

  if (softDemo->m_drag)
    {
      const int x = softDemo->m_lastmousepos[0];
      const int y = softDemo->m_lastmousepos[1];
      float rf[3];
      softDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraPosition(rf);
      float target[3];
      softDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraTargetPosition(target);
      btVector3 cameraTargetPosition(target[0], target[1], target[2]);

      const btVector3 cameraPosition(rf[0], rf[1], rf[2]);
      const btVector3 rayFrom = cameraPosition;

      const btVector3 rayTo = softDemo->getRayTo(x, y);
      const btVector3 rayDir = (rayTo - rayFrom).normalized();
      const btVector3 N = (cameraTargetPosition - cameraPosition).normalized();
      const btScalar O = btDot(softDemo->m_impact, N);
      const btScalar den = btDot(N, rayDir);
      if ((den * den) > 0)
	{
	  const btScalar num = O - btDot(N, rayFrom);
	  const btScalar hit = num / den;
	  if ((hit > 0) && (hit < 1500))
	    {
	      softDemo->m_goal = rayFrom + rayDir * hit;
	    }
	}
      btVector3 delta = softDemo->m_goal - softDemo->m_node->m_x;
      static const btScalar maxdrag = 10;
      if (delta.length2() > (maxdrag * maxdrag))
	{
	  delta = delta.normalized() * maxdrag;
	}
      softDemo->m_node->m_v += delta / timeStep;
    }
}

//
// ImplicitShape
//

//
struct ImplicitSphere : btSoftBody::ImplicitFn
{
  btVector3 center;
  btScalar sqradius;
  ImplicitSphere() {}
  ImplicitSphere(const btVector3& c, btScalar r) : center(c), sqradius(r * r) {}
  btScalar Eval(const btVector3& x)
  {
    return ((x - center).length2() - sqradius);
  }
};


//
// Random
//

static inline btScalar UnitRand()
{
  return (rand() / (btScalar)RAND_MAX);
}

static inline btScalar SignedUnitRand()
{
  return (UnitRand() * 2 - 1);
}

static inline btVector3 Vector3Rand()
{
  const btVector3 p = btVector3(SignedUnitRand(), SignedUnitRand(), SignedUnitRand());
  return (p.normalized());
}

//
// Rb rain
//
static void Ctor_RbUpStack(Nursing* pdemo, int count)
{
  float mass = 10;

  btCompoundShape* cylinderCompound = new btCompoundShape;
  btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(4, 1, 1));
  btCollisionShape* boxShape = new btBoxShape(btVector3(4, 1, 1));
  btTransform localTransform;
  localTransform.setIdentity();
  cylinderCompound->addChildShape(localTransform, boxShape);
  btQuaternion orn(SIMD_HALF_PI, 0, 0);
  localTransform.setRotation(orn);
  //	localTransform.setOrigin(btVector3(1,1,1));
  cylinderCompound->addChildShape(localTransform, cylinderShape);

  btCollisionShape* shape[] = {cylinderCompound,
    new btBoxShape(btVector3(1, 1, 1)),
    new btSphereShape(1.5)

  };
  static const int nshapes = sizeof(shape) / sizeof(shape[0]);
  for (int i = 0; i < count; ++i)
    {
      btTransform startTransform;
      startTransform.setIdentity();
      startTransform.setOrigin(btVector3(0, 2 + 6 * i, 0));
      pdemo->createRigidBody(mass, startTransform, shape[i % nshapes]);
      //pdemo->createRigidBody(mass,startTransform,shape[0]);
    }
}

//
// Big plate
//
static btRigidBody* Ctor_BigPlate(Nursing* pdemo, btScalar mass = 15, btScalar height = 4)
{
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, height, 0.5));
  btRigidBody* body = pdemo->createRigidBody(mass, startTransform, new btBoxShape(btVector3(5, 1, 5)));
  body->setFriction(1);
  return (body);
}

//
// Softbox
//
static btSoftBody* Ctor_SoftBox(Nursing* pdemo, const btVector3& p, const btVector3& s)
{
  const btVector3 h = s * 0.5;
  const btVector3 c[] = {p + h * btVector3(-1, -1, -1),
    p + h * btVector3(+1, -1, -1),
    p + h * btVector3(-1, +1, -1),
    p + h * btVector3(+1, +1, -1),
    p + h * btVector3(-1, -1, +1),
    p + h * btVector3(+1, -1, +1),
    p + h * btVector3(-1, +1, +1),
    p + h * btVector3(+1, +1, +1)};
  btSoftBody* psb = btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo, c, 8);
  psb->generateBendingConstraints(2);
  pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

  return (psb);
}

//
// SoftBoulder
//
static btSoftBody* Ctor_SoftBoulder(Nursing* pdemo, const btVector3& p, const btVector3& s, int np, int id)
{
  btAlignedObjectArray<btVector3> pts;
  if (id) srand(id);
  for (int i = 0; i < np; ++i)
    {
      pts.push_back(Vector3Rand() * s + p);
    }
  btSoftBody* psb = btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo, &pts[0], pts.size());
  psb->generateBendingConstraints(2);
  pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

  return (psb);
}


//
// 100kg Stanford's bunny
//

void Nursing::setDrawClusters(bool drawClusters)
{
  if (drawClusters)
    {
      getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() | fDrawFlags::Clusters);
    }
  else
    {
      getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() & (~fDrawFlags::Clusters));
    }
}

//
void Nursing::mouseMotionFunc(int x, int y)
{
  if (m_node && (m_results.fraction < 1.f))
    {
      if (!m_drag)
	{
#define SQ(_x_) (_x_) * (_x_)
	  if ((SQ(x - m_lastmousepos[0]) + SQ(y - m_lastmousepos[1])) > 6)
	    {
	      m_drag = true;
	    }
#undef SQ
	}
      if (m_drag)
	{
	  m_lastmousepos[0] = x;
	  m_lastmousepos[1] = y;
	}
    }
}


void Nursing::initPhysics()
{
  ///create concave ground mesh

  m_guiHelper->setUpAxis(1);
  // m_guiHelper->setUpAxis(2);
  //	m_azi = 0;


  btCollisionShape* groundShape = 0;
  {
    int i;
    int j;

    const int NUM_VERTS_X = 30;
    const int NUM_VERTS_Y = 30;
    const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;
    const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

    gGroundVertices = new btVector3[totalVerts];
    gGroundIndices = new int[totalTriangles * 3];

    btScalar offset(-50);

    for (i = 0; i < NUM_VERTS_X; i++)
      {
	for (j = 0; j < NUM_VERTS_Y; j++)
	  {
	    gGroundVertices[i + j * NUM_VERTS_X].setValue((i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE,
							  //0.f,
							  waveheight * sinf((float)i) * cosf((float)j + offset),
							  (j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
	  }
      }

    int vertStride = sizeof(btVector3);
    int indexStride = 3 * sizeof(int);

    int index = 0;
    for (i = 0; i < NUM_VERTS_X - 1; i++)
      {
	for (int j = 0; j < NUM_VERTS_Y - 1; j++)
	  {
	    gGroundIndices[index++] = j * NUM_VERTS_X + i;
	    gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
	    gGroundIndices[index++] = j * NUM_VERTS_X + i + 1;
	    ;

	    gGroundIndices[index++] = j * NUM_VERTS_X + i;
	    gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i;
	    gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
	  }
      }

    btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
										   gGroundIndices,
										   indexStride,
										   totalVerts, (btScalar*)&gGroundVertices[0].x(), vertStride);

    bool useQuantizedAabbCompression = true;

    groundShape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);
    groundShape->setMargin(0.5);
  }

  m_collisionShapes.push_back(groundShape);

  btCollisionShape* groundBox = new btBoxShape(btVector3(100, CUBE_HALF_EXTENTS, 100));
  m_collisionShapes.push_back(groundBox);

  m_dispatcher = 0;

  ///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
  m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

  ////////////////////////////
  ///Register softbody versus softbody collision algorithm

  ///Register softbody versus rigidbody collision algorithm

  ////////////////////////////

  btVector3 worldAabbMin(-1000, -1000, -1000);
  btVector3 worldAabbMax(1000, 1000, 1000);

  m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

  m_softBodyWorldInfo.m_broadphase = m_broadphase;

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

  m_solver = solver;

  btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

  static bool once = true;
  if (once)
    {
      once = false;
      initCL(0, 0);
    }

  if (g_openCLSIMDSolver)
    delete g_openCLSIMDSolver;
  if (g_softBodyOutput)
    delete g_softBodyOutput;

  if (1)
    {
      g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
      //	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
      g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
    }

  softBodySolver = g_openCLSIMDSolver;
  g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

  // btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
  // m_dynamicsWorld = world;
  createEmptyDynamicsWorld();
  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);

  m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);

  m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
  m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
  // m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
  // m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  //	clientResetScene();

  m_softBodyWorldInfo.m_sparsesdf.Initialize();
  //	clientResetScene();

  //create ground object
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(0, 0, 0));

  btCollisionObject* newOb = new btCollisionObject();
  newOb->setWorldTransform(tr);
  newOb->setInterpolationWorldTransform(tr);

  newOb->setCollisionShape(m_collisionShapes[1]);


  m_dynamicsWorld->addCollisionObject(newOb);

  m_softBodyWorldInfo.m_sparsesdf.Reset();


  m_softBodyWorldInfo.air_density = (btScalar)1.2;
  m_softBodyWorldInfo.water_density = 0;
  m_softBodyWorldInfo.water_offset = 0;
  m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
  m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);

  m_autocam = false;
  m_raycast = false;
  m_cutting = false;
  m_results.fraction = 1.f;

  BedFrame::registerModel(this);
  Mattress::registerModel(this);

  // ----------------------------------------------------------------------------------
  // create humanoid models
#if 0
  createEmptyDynamicsWorld();

  //MuJoCo uses a slightly different collision filter mode, use the FILTER_GROUPAMASKB_OR_GROUPBMASKA2
  //@todo also use the modified collision filter for raycast and other collision related queries
  m_filterCallback->m_filterMode = FILTER_GROUPAMASKB_OR_GROUPBMASKA2;

  //m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);
#endif

  if (m_guiHelper->getParameterInterface())
  // if (m_guiHelper->getAppInterface())
  {
    b3Printf("OK\n");
    SliderParams slider("Gravity", &m_grav);
    slider.m_minVal = -10;
    slider.m_maxVal = 10;
    // Gravityのスライダーを設定
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
  }

  // OK
  int flags = 0;
  b3BulletDefaultFileIO fileIO;
  BulletMJCFImporter importer(m_guiHelper, 0, &fileIO, flags);
  MyMJCFLogger logger;
  bool result = importer.loadMJCF(m_fileName, &logger);
  if (result)
  {
    btTransform rootTrans;
    rootTrans.setIdentity();

    // OK
    for (int m = 0; m < importer.getNumModels(); m++)
    {
      importer.activateModel(m);

      // normally used with PhysicsServerCommandProcessor that allocates unique ids to multibodies,
      // emulate this behavior here:
      importer.setBodyUniqueId(m);

      btMultiBody* mb = 0;

      //todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
      //int rootLinkIndex = importer.getRootLinkIndex();
      //b3Printf("urdf root link index = %d\n",rootLinkIndex);
      MyMultiBodyCreator creation(m_guiHelper);

      rootTrans.setIdentity();
      importer.getRootTransformInWorld(rootTrans);

      ConvertURDF2Bullet(importer, creation, rootTrans, m_dynamicsWorld, m_useMultiBody, importer.getPathPrefix(), CUF_USE_MJCF);

      mb = creation.getBulletMultiBody();

      if (mb)
      {
	std::string* name = new std::string(importer.getLinkName(importer.getRootLinkIndex()));m_nameMemory.push_back(name);
#ifdef TEST_MULTIBODY_SERIALIZATION
	s->registerNameForPointer(name->c_str(), name->c_str());
#endif  //TEST_MULTIBODY_SERIALIZATION
	mb->setBaseName(name->c_str());
	mb->getBaseCollider()->setCollisionFlags(mb->getBaseCollider()->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);

	//create motors for each btMultiBody joint
	int numLinks = mb->getNumLinks();
	for (int i = 0; i < numLinks; i++) // numLinks = 33
	{
	  int mbLinkIndex = i;
	  int urdfLinkIndex = creation.m_mb2urdfLink[mbLinkIndex];

	  std::string* jointName = new std::string(importer.getJointName(urdfLinkIndex));
	  std::string* linkName = new std::string(importer.getLinkName(urdfLinkIndex).c_str());
#ifdef TEST_MULTIBODY_SERIALIZATION
	  s->registerNameForPointer(jointName->c_str(), jointName->c_str());
	  s->registerNameForPointer(linkName->c_str(), linkName->c_str());
#endif  //TEST_MULTIBODY_SERIALIZATION
	  m_nameMemory.push_back(jointName);
	  m_nameMemory.push_back(linkName);
	  mb->getLinkCollider(i)->setCollisionFlags(mb->getBaseCollider
	  ()->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);

	  mb->getLink(i).m_linkName = linkName->c_str();
	  mb->getLink(i).m_jointName = jointName->c_str();
	  m_data->m_mb = mb;
	  if (mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::eRevolute || mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::ePrismatic)
	  {
	    if (m_data->m_numMotors < MAX_NUM_MOTORS)
	    {
	      char motorName[1024];
	      sprintf(motorName, "%s q ", jointName->c_str());
	      btScalar* motorPos = &m_data->m_motorTargetPositions[m_data->m_numMotors];
	      *motorPos = 0.f;
	      SliderParams slider(motorName, motorPos);
	      slider.m_minVal = -4;
	      slider.m_maxVal = 4;
	      slider.m_clampToIntegers = false;
	      slider.m_clampToNotches = false;
	      // b3Printf("param = %f\n", m_guiHelper->getParameterInterface());
	      // JointMotorのスライダーを設定
	      // m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	      float maxMotorImpulse = 5.f;
	      btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb, mbLinkIndex, 0, 0, maxMotorImpulse);
	      motor->setErp(0.1);
	      //motor->setMaxAppliedImpulse(0);
	      m_data->m_jointMotors[m_data->m_numMotors] = motor;
	      m_dynamicsWorld->addMultiBodyConstraint(motor);
	      m_data->m_numMotors++;
	    }
	  }
        }
      }
    }
  }

  m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Nursing::exitPhysics()
{
  //cleanup in the reverse order of creation/initialization

  //remove the rigidbodies from the dynamics world and delete them
  int i;
  for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
	{
	  delete body->getMotionState();
	}
      m_dynamicsWorld->removeCollisionObject(obj);
      delete obj;
    }

  //delete collision shapes
  for (int j = 0; j < m_collisionShapes.size(); j++)
    {
      btCollisionShape* shape = m_collisionShapes[j];
      m_collisionShapes[j] = 0;
      delete shape;
    }

  //delete dynamics world
  delete m_dynamicsWorld;
  m_dynamicsWorld = 0;

  //delete solver
  delete m_solver;

  //delete broadphase
  delete m_broadphase;

  //delete dispatcher
  delete m_dispatcher;

  delete m_collisionConfiguration;
}

// ----------------------------------------------------------------------------------
// step Simulation
void Nursing::stepSimulation(float deltaTime)
{
if (m_dynamicsWorld)
{
  // btVector3 gravity(0, 0, -10);
  // gravity[m_upAxis] = m_grav;
  // m_dynamicsWorld->setGravity(gravity);

  for (int i = 0; i < m_data->m_numMotors; i++)
  {
    if (m_data->m_jointMotors[i])
    {
      btScalar pos = m_data->m_motorTargetPositions[i];

      int link = m_data->m_jointMotors[i]->getLinkA();
      btScalar lowerLimit = m_data->m_mb->getLink(link).m_jointLowerLimit;
      btScalar upperLimit = m_data->m_mb->getLink(link).m_jointUpperLimit;
      if (lowerLimit < upperLimit)
      {
	btClamp(pos, lowerLimit, upperLimit);
      }
      m_data->m_jointMotors[i]->setPositionTarget(pos);
    }
#if 0
    if (m_data->m_generic6DofJointMotors[i])
    {
      GenericConstraintUserInfo* jointInfo = (GenericConstraintUserInfo*)m_data->m_generic6DofJointMotors[i]->getUserConstraintPtr();
      m_data->m_generic6DofJointMotors[i]->setTargetVelocity(jointInfo->m_jointAxisIndex, m_data->m_motorTargetPositions[i]);
    }
#endif
  }

  //the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
  m_dynamicsWorld->stepSimulation(deltaTime, 10, 1. / 240.);
  }
}
// ----------------------------------------------------------------------------------

class CommonExampleInterface* NursingCreateFunc(struct CommonExampleOptions& options)
{
  current_demo = options.m_option;
  return new Nursing(options.m_guiHelper, options.m_option, options.m_fileName);
}
