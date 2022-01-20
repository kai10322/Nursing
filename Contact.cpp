/*
  Bullet Continuous Collision Detection and Physics Library
  Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

  This software is provided 'as-is', without any express or implied warranty.
  In no event will the authors be held liable for any damages arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose, 
 MAX_CONTACT_FORCi including commercial applications, and to alter it and redistribute it freely, 
  subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson

#define CREATE_RIGID_BOX
#define MAX_CONTACT_FORCE 80 // [N] // humanoidの全質量は約41kgなので
#define MAX_SOFTBODY_IMPULSE 0.01 // [N*s(TimeStep)]
#define MAX_SOFTBODY_FORCE 0.5 // [N] // 2.4, 0.8
#define MAX_JOINTMOTOR_TORQUE 50 // [N*m] // 

#define NOT_ESTIMATED 100000

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h" //

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
// #include <BulletCollision/CollisionDispatch/btCollisionObject.h>
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

// #include <ExampleBrowser/OpenGLExampleBrowser.h>
// #include <ExampleBrowser/OpenGLGuiHelper.h>

// ---------------------------------------
// include standard c++ libraly
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <map>
// ---------------------------------------

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
// class btConstraintSolver;
class btMultiBodyConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftMultiBodynamicsWorld;

// broadphaseに使う
btOverlappingPairCache* m_pairCache = 0;

#include "CommonInterfaces/CommonMultiBodyBase.h"

#include "BedFrame.h"
#include "Mattress.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

//static bool sDemoMode = false;

// user index
int UserId = 10;
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

btVector3 white = btVector3(1, 1, 1);
btVector3 red = btVector3(1, 0, 0);
btVector3 green = btVector3(0, 1, 0);
btVector3 blue = btVector3(0, 0, 1);
btVector3 yellow = btVector3(1, 1, 0);
btVector3 black = btVector3(0, 0, 0);

//

struct TetraCube
{
#include "cube.inl"
};

// ---------------------
// variable to store mjcf file names
static btAlignedObjectArray<std::string> gMCFJFileNameArray;
// --------------------

btAlignedObjectArray<struct ImportMJCFInternalData*> m_datas;

// ----------------------------------------------------------------------------------
// constructor 
Nursing::Nursing(struct GUIHelperInterface* helper)
    : CommonMultiBodyBase(helper),
      m_drag(false)
{
  m_data = new ImportMJCFInternalData;

  m_useMultiBody = true;

  static int count = 0;
#if 0
  if (fileName)
  {
    setFileName(fileName);
  }
  else
#endif



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
   	    gMCFJFileNameArray.push_back("./ground.xml");

   	    // gMCFJFileNameArray.push_back("./humanoid_model/original_humanoid_noLimit.xml");

	  // gMCFJFileNameArray.push_back("./humanoid2.xml");
	  // gMCFJFileNameArray.push_back("./humanoid_SoftTest.xml");
    }
    int numFileNames = gMCFJFileNameArray.size();

    if (count >= numFileNames)
    {
	    count = 0;
    }
    sprintf(m_fileName, "%s", gMCFJFileNameArray[count++].c_str());
  }

  /*
  for(int i = 0; i < gMCFJFileNameArray.size(); i++){
    b3Printf("gMCFJFileNameArray[%d] = %s\n", i, gMCFJFileNameArray[i].c_str());
  }
  */
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
  // btQuaternion orn(SIMD_HALF_PI, 0, 0);
  btQuaternion orn(0, 0, 0);
  // localTransform.setRotation(orn);
  localTransform.setOrigin(btVector3(0,0,10));
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
// Rigid Sphere
//
static btRigidBody* createRigidSphere(Nursing* pdemo, btScalar mass = 1.0, btScalar radius = 0.06)
{
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, 1.0));
  btRigidBody* body = pdemo->createRigidBody(mass, startTransform, new btSphereShape(radius));
  int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
  int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  pdemo->m_dynamicsWorld->addRigidBody(body, collisionFilterGroup, collisionFilterMask);
  return (body);
}


//
// Rigid Box 
//
static btRigidBody* createRigidBox(Nursing* pdemo, btScalar mass = 1.0, btScalar edge_length = 0.5)
{
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, -2, 1.0));
  btRigidBody* body = pdemo->createRigidBody(mass, startTransform, new btBoxShape(btVector3(edge_length, edge_length, edge_length)));
  int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
  int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  pdemo->m_dynamicsWorld->addRigidBody(body, collisionFilterGroup, collisionFilterMask);
  return (body);
}

//
// Rigid Bed Box
//
static btRigidBody* createBedBox(Nursing* pdemo, btScalar mass = 0.0, btScalar base_size = 0.5)
{
  //
  // 	Create Rigid Mattress	//
  //
  // 190:100:40[cm]
  btScalar width = base_size;
  btScalar height = base_size * 0.7;
  btScalar depth = base_size * 1.9;
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, height));
  btRigidBody* body = pdemo->createRigidBody(mass, startTransform, new btBoxShape(btVector3(depth, width, height)));
  body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

  // int collisionFilterGroup = int(btBroadphaseProxy::StaticFilter);
  // int collisionFilterMask = int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
  int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
  int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  pdemo->m_dynamicsWorld->addRigidBody(body, collisionFilterGroup, collisionFilterMask);
  return (body);
}

//
// Cretate Compound Bed 
//
static btRigidBody* createCompoundBed(Nursing* pdemo, btScalar mass = 0.0, btScalar base_size = 0.5f)
{
  btCompoundShape* BedShape = new btCompoundShape();
  // btScalar curve_radius = 0.05;
  btScalar margin = 0.05;
 
  //
  // 	Create Mattress	Shape	//
  //
  //	Create Main Body	//
  // 190:100:40[cm]
  btScalar width = base_size - (margin/2);
  btScalar height = base_size * 0.3f; // 0.4f
  // btScalar height = base_size * 0.3;
  btScalar depth = base_size * 1.9f - (margin/2.f);
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(btVector3(0, 0, height));
  btBoxShape* mainShape = new btBoxShape(btVector3(depth, width, height));
  BedShape->addChildShape(trans, mainShape);
  

  // 	Create Edge Shape	//
  btScalar edge_radius = margin;
  btScalar edge_height1 = 2 * width; // ベッドの短辺
  btScalar edge_height2 = 2 * depth; // ベッドの長辺
  // Edge1
  trans.setIdentity();
  trans.setOrigin(btVector3(depth, 0, height*2 - margin));
  btCapsuleShape* edge1 = new btCapsuleShape(edge_radius, edge_height1);
  BedShape->addChildShape(trans, edge1);
  // Edge2
  trans.setIdentity();
  trans.setOrigin(btVector3(-depth, 0, height*2 - margin));
  btCapsuleShape* edge2 = new btCapsuleShape(edge_radius, edge_height1);
  BedShape->addChildShape(trans, edge2);
  // Edge3
  trans.setIdentity();
  trans.setOrigin(btVector3(0, width, height*2 - margin));
  btCapsuleShapeX* edge3 = new btCapsuleShapeX(edge_radius, edge_height2);
  BedShape->addChildShape(trans, edge3);
  // Edge4
  trans.setIdentity();
  trans.setOrigin(btVector3(0, -width, height*2 - margin));
  btCapsuleShapeX* edge4 = new btCapsuleShapeX(edge_radius, edge_height2);
  BedShape->addChildShape(trans, edge4);

#if 0
  // 	Create Corner Shape	//
  btScalar corner_radius = margin;
  btSphereShape* corner = new btSphereShape(corner_radius);
  // Corner1	
  trans.setIdentity();
  trans.setOrigin(btVector3(depth - corner_radius, width - corner_radius, height*2));
  BedShape->addChildShape(trans, corner);
  // Corner2	
  trans.setOrigin(btVector3(0, -(width - margin), height*2));
  // BedShape->addChildShape(trans, edge);
  // Corner3	
  trans.setOrigin(btVector3(0, depth - margin, height*2));
  // BedShape->addChildShape(trans, edge);
  // Corner4	 
  trans.setOrigin(btVector3(0, -(depth + margin), height*2));
  // BedShape->addChildShape(trans, edge);
#endif

  // 
  // 	Create Face Shape	//
  //
  btScalar face_z = height - (margin/2); 
  // 	短辺側	 //
  // Face1
  btScalar face_depth = margin / 2; 
  btScalar face_width = width; 
  btScalar face_height = height - (margin/2);
  trans.setIdentity();
  trans.setOrigin(btVector3(depth + (margin/2), 0, face_z));
  btBoxShape* FaceShape1 = new btBoxShape(btVector3(face_depth, face_width, face_height));
  BedShape->addChildShape(trans, FaceShape1);
  // Face2
  trans.setIdentity();
  trans.setOrigin(btVector3(-(depth + (margin/2)), 0, face_z));
  BedShape->addChildShape(trans, FaceShape1);
  // 	長辺側	  //
  // Face3
  face_depth = depth; 
  face_width = margin/2; 
  trans.setIdentity();
  trans.setOrigin(btVector3(0, width + (margin/2), face_z));
  btBoxShape* FaceShape3 = new btBoxShape(btVector3(face_depth, face_width, face_height));
  BedShape->addChildShape(trans, FaceShape3);
  // Face4
  trans.setIdentity();
  trans.setOrigin(btVector3(0, -(width + (margin/2)), face_z));
  BedShape->addChildShape(trans, FaceShape3);

  //	Create Bed Leg	//
  btScalar leg_radius = margin;
  btScalar Ground_Height = 0.2;
  btScalar leg_height = height + (Ground_Height/2);
  btCylinderShapeZ *leg = new btCylinderShapeZ(btVector3(leg_radius, leg_radius, leg_height));
  btScalar e = 0.00;
  btScalar leg_z = height - (Ground_Height/2) - margin;
  // leg1
  trans.setIdentity();
  trans.setOrigin(btVector3(depth, width, leg_z));
  BedShape->addChildShape(trans, leg);
  // leg2
  trans.setIdentity();
  trans.setOrigin(btVector3(-depth, width, leg_z));
  BedShape->addChildShape(trans, leg);
  // leg3
  trans.setIdentity();
  trans.setOrigin(btVector3(depth, -width, leg_z));
  BedShape->addChildShape(trans, leg);
  // leg4
  trans.setIdentity();
  trans.setOrigin(btVector3(-depth, -width, leg_z));
  BedShape->addChildShape(trans, leg);

  // 	Create Compound Shape 	//
  btScalar body_base = height;

  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, body_base));
  btRigidBody* BedBody = pdemo->createRigidBody(mass, startTransform, BedShape);
  BedBody->setFriction(1);;
  BedBody->setRollingFriction(1);
  BedBody->setCollisionFlags(BedBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

  // int collisionFilterGroup = int(btBroadphaseProxy::StaticFilter);
  // int collisionFilterMask = int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
  int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
  int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  pdemo->m_dynamicsWorld->addRigidBody(BedBody, collisionFilterGroup, collisionFilterMask);
  return (BedBody);
}
//
// Softbox
//
static btSoftBody* Ctor_SoftBox(Nursing* pdemo, const btVector3& p, const btVector3& s)
{
  const btVector3 h = s * 0.4;
  const btVector3 c[] = {p + h * btVector3(-1, -1, -1),
    p + h * btVector3(+1, -1, -1),
    p + h * btVector3(-1, +1, -1),
    p + h * btVector3(+1, +1, -1),
    p + h * btVector3(-1, -1, +1),
    p + h * btVector3(+1, -1, +1),
    p + h * btVector3(-1, +1, +1),
    p + h * btVector3(+1, +1, +1)};
  btSoftBody* psb = btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo, c, 8);
  btScalar Margin = psb->getCollisionShape()->getMargin();
  b3Printf("Margin = %f\n", Margin);
  psb->getCollisionShape()->setMargin(0.1);
  b3Printf("Margin = %f\n", Margin);
  psb->generateBendingConstraints(2);
  pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

  int flag = psb->getCollisionFlags();
  psb->setCollisionFlags(psb->getCollisionFlags() | btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING);
  b3Printf("flags = %d\n", flag);
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

// 
// Tetra Cube
//
static btSoftBody* TetraCube(Nursing* pdemo, const btVector3& p, const btVector3& s)
{
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(
		  pdemo->m_softBodyWorldInfo,
		  TetraCube::getElements(),0,TetraCube::getNodes(),
		  false, true, true);

  psb->scale(btVector3(s.x()*0.65, s.y()*0.75, s.z()*0.7));
  psb->translate(btVector3(p.x(), p.y(), p.z()));
  // psb->setVolumeMass(4500);
  psb->setVolumeDensity(45); // 45
  psb->m_cfg.piterations = 10; // 200
  // psb->m_cfg.viterations = 100;
  // psb->m_cfg.diterations = 100;

  psb->generateClusters(8);
  // psb->m_cfg.kSR_SPLT_CL = 1.0; // Soft vs rigid impulse split (cluster only)
  // psb->m_cfg.citerations = 10;

  psb->getCollisionShape()->setMargin(0.01); // 0.13
  // psb->getCollisionShape()->setMargin(0.05);
  psb->m_materials[0]->m_kLST = 0.4; // 0.4
  psb->m_materials[0]->m_kVST = 0.4; // 0.3
  psb->m_cfg.kKHR = 0.8;
  psb->m_cfg.kCHR = 0.1;
  psb->m_cfg.kSSHR_CL = 0.1;
  psb->m_cfg.kSS_SPLT_CL = 1.;
  // psb->m_cfg.kDF = 1;
  psb->generateBendingConstraints(2);
  // psb->m_cfg.kMT = 0.1; // 元の形状を保とうとする力を働かせる // 0.5
  // psb->m_cfg.kVC = 1.0; // 体積維持係数(体積一定にする力) // 40.0
  psb->m_sleepingThreshold = 0;
  // psb->setPose(true, false);
  pdemo->getSoftDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
  pdemo->getSoftDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-5;
  pdemo->getSoftDynamicsWorld()->getSolverInfo().m_numIterations = 100;
  psb->m_cfg.collisions = 
	  // btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;
	  btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS;

  pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
  // btSoftBodyHelpers::generateBoundaryFaces(psb);

  btCollisionObject* link_col = btSoftBody::upcast(psb);
  //  link_col->setCcdSweptSphereRadius(0.9);
  // link_col->setCcdMotionThreshold(0.0001);
  // btCollisionShape* col_shape = link_col->getCollisionShape();
  // col_shape->setMargin(0.35);
  
  return (psb);
}

//
//Cloth
//
// static btSoftBody* Cloth(Nursing* pdemo, const btScalar& s)
static btSoftBody* Cloth(Nursing* pdemo)
{
	// btScalar margin = 0.13; // 衝突マージン(上から落とす時)
	btScalar margin = 0.07; // 衝突マージン // 0.05 or 0.1
        btScalar Bed_height = 0.5*0.38*2 + margin + 0.2; // Bed Height : 0.24*2, Ground Height : 0.2
	btScalar h = 1.2;
        btScalar w = 0.7;
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(
			pdemo->m_softBodyWorldInfo, 
			btVector3(-h*1., -w*1., 0), btVector3(+h*1., -w*1., 0),
                        btVector3(-h*1., +w*1., 0),btVector3(+h*1., +w*1., 0),
                        51, 51, // 17,17, // 51, 51 // 62, 62
                        0, // 1 + 2 + 4 + 8
			true);

        btSoftBody::Material* pm = psb->appendMaterial();
        psb->m_materials[0]->m_kLST = 0.3; // 0.3 or 0.6
        psb->m_materials[0]->m_kAST = 0.1; // 
        pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
        // psb->generateBendingConstraints(2, pm);
        psb->setTotalMass(20.0); // 15kgぐらいなら布を浮かせていても破れない(RigidSphere時)
        // psb->setTotalMass(615.0);  // 600 // 1kg 
        // psb->setVolumeDensity(1);
	psb->translate(btVector3(0, 0, Bed_height));
	psb->m_cfg.kDF = 1;
        psb->m_cfg.kCHR = 0.1; // 0.3
        psb->m_cfg.kKHR = 0.3; // 0.3
        // psb->m_cfg.kSRHR_CL = 1;
        // psb->m_cfg.kSR_SPLT_CL = 0;
        // psb->m_cfg.kSSHR_CL = 0.1; // 0.3
        // psb->m_cfg.kSS_SPLT_CL = 1.;
	// psb->m_cfg.kDP = 0.1;
	// psb->m_cfg.kLF = 1.0; // 1.0
	// psb->m_cfg.kDG = 0.0;
	// psb->m_cfg.kPR = 0.5;
        // psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;
        psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::SDF_RS; // RigidBodyは布の上にある
        // psb->generateClusters(8); // 0同士はだめっぽい
        psb->getCollisionShape()->setMargin(margin); // 0.135 // BedFrameありなら0.08でもいけそう
        psb->m_sleepingThreshold = 0;
        pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
        // psb->setCollisionFlags(0);

	// psb->setWindVelocity(btVector3(4.0, 0.0, 0.0));
	// psb->setSpringStiffness(.1); // 強度が上がる

         psb->m_cfg.piterations = 100;
         psb->m_cfg.viterations = 100;
         psb->m_cfg.diterations = 100;
        // psb->m_cfg.citerations = 100;

        psb->setFriction(1);
        psb->setRollingFriction(1);

        pdemo->getSoftDynamicsWorld()->getSolverInfo().m_splitImpulse = true;

        pdemo->m_cutting = false;
	return (psb);
}

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

struct MyConvertPointerSizeT
{
        union {
                const void* m_ptr;
                size_t m_int;
        };
};
bool shapePointerCompareFunc(const btCollisionObject* colA, const btCollisionObject* colB)
{
        MyConvertPointerSizeT a, b;
        a.m_ptr = colA->getCollisionShape();
        b.m_ptr = colB->getCollisionShape();
        return (a.m_int < b.m_int);
}

int Nursing::createCheckeredTexture()
{
  int texWidth = 1024;
  int texHeight = 1024;
  btAlignedObjectArray<unsigned char> texels;
  texels.resize(texWidth * texHeight * 3);
  for (int i = 0; i < texWidth * texHeight * 3; i++)
    texels[i] = 255;

  int texId = m_guiHelper->registerTexture(&texels[0], texWidth, texHeight);
  return texId;
}

void Nursing::autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld, btVector4 rgba)
{
//sort the collision objects based on collision shape, the gfx library requires instances that re-use a shape to be added after eachother

  btAlignedObjectArray<btCollisionObject*> sortedObjects;
  sortedObjects.reserve(rbWorld->getNumCollisionObjects());
  for (int i = 0; i < rbWorld->getNumCollisionObjects(); i++)
  {
    btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
    sortedObjects.push_back(colObj);
  }
  sortedObjects.quickSort(shapePointerCompareFunc);
  int whiteTextureId = createCheckeredTexture();
  // b3Printf("whiteTextureId = %d\n", whiteTextureId);
  for (int i = 0; i < sortedObjects.size(); i++)
  {
    btCollisionObject* colObj = sortedObjects[i];
    //btRigidBody* body = btRigidBody::upcast(colObj);
    //does this also work for btMultiBody/btMultiBodyLinkCollider?
    btSoftBody* sb = btSoftBody::upcast(colObj);
    if (sb)
    {
      colObj->getCollisionShape()->setUserPointer(sb);
    }
    m_guiHelper->createCollisionShapeGraphicsObject(colObj->getCollisionShape());
    int shapeIndex = colObj->getUserIndex();
    int colorIndex = colObj->getBroadphaseHandle()->getUid() & 3;
    btVector4 color = rgba;
    // color = sColors[colorIndex];

    if (colObj->getCollisionShape()->getShapeType() == STATIC_PLANE_PROXYTYPE)
    {
      color.setValue(1, 1, 1, 1);
    }

    m_guiHelper->replaceTexture(i, whiteTextureId); // 布のTextureは変わった
   

    int texWidth = 1024;
    int texHeight = 1024;
    btAlignedObjectArray<unsigned char> texels;
    texels.resize(texWidth * texHeight * 3);
    for (int i = 0; i < texWidth * texHeight * 3; i++)
      texels[i] = 255;
    m_guiHelper->changeTexture(2, &texels[0], texWidth, texHeight);

#if 0
    if(colObj->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT)
    {
      btRigidBody* rb = btRigidBody::upcast(colObj);
      sb->appendAnchor(0, rb);
    }
#endif
    m_guiHelper->createCollisionObjectGraphicsObject(colObj, color);
  }
}

btAlignedObjectArray<std::string> MotorJointNames;
void Nursing::initPhysics()
{
  ///create concave ground mesh

  // m_guiHelper->m_data->m_checkedTextureGrey = 100;
  // getOpenGLGuiHelper()->m_data->m_checkedTextureGrey = 100;

  // m_guiHelper->setUpAxis(1);
  m_guiHelper->setUpAxis(2);
  //	m_azi = 0;

  /*
  // Nursing::setDrawClusters(true);
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
  */

  m_dispatcher = 0;

  ///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
  m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
  // m_collisionConfiguration = new btDefaultCollisionConfiguration();

  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

  ////////////////////////////
  ///Register softbody versus softbody collision algorithm

  ///Register softbody versus rigidbody collision algorithm

  ////////////////////////////

  btVector3 worldAabbMin(-1000, -1000, -1000);
  btVector3 worldAabbMax(1000, 1000, 1000);

  // btVector3 worldAabbMin(-100, -100, -100);
  // btVector3 worldAabbMax(100, 100, 100);

  m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);
#if 0 
  m_filterCallback = new MyOverlapFilterCallback2();
  m_pairCache = new btHashedOverlappingPairCache();
  m_pairCache->setOverlapFilterCallback(m_filterCallback);
  m_broadphase = new btDbvtBroadphase(m_pairCache);  //btSimpleBroadphase();
#endif
  m_softBodyWorldInfo.m_broadphase = m_broadphase;

  // btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
  // m_solver = solver;
  
  btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();
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
  // createEmptyDynamicsWorld(); // MultiBodyDynamicsWorldが生成される

  btMultiBodyDynamicsWorld* world = new btSoftMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
  m_dynamicsWorld = world; // m_dynamicsはbyMultiBodyDynamicsWorld型
  // -------------------------------------------------
  // check world type
  // const std::type_info& info = typeid(m_dynamicsWorld);
  // const char* typeName = info.name();
  // b3Printf("typeName = %s\n", typeName);
  // -------------------------------------------------

  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(0);

#if 0 
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(
      	  btIDebugDraw::DBG_DrawConstraints 
      	+ btIDebugDraw::DBG_DrawContactPoints 
	+ btIDebugDraw::DBG_DrawAabb 
	+ btIDebugDraw::DBG_DrawConstraintLimits 
	+ btIDebugDraw::DBG_DrawWireframe
	);
#endif

  // m_dynamicsWorld->debugDrawWorld();
  // int debugMode = m_dynamicsWorld->getDebugDrawer()->getDebugMode(); 
  // b3Printf("	=====> debugMode = %d\n", debugMode);
  m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);

  m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
  m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
  m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
  // m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  //	clientResetScene();

  m_softBodyWorldInfo.m_sparsesdf.Initialize();
  //	clientResetScene();

#ifndef DEBUG_HUMANOID  //create ground object
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(0, 0, -1.5));
  btQuaternion q; q.setRotation(btVector3(1, 0, 0), SIMD_PI / 2);
  tr.setRotation(q);

  btCollisionObject* newOb = new btCollisionObject();
  newOb->setWorldTransform(tr);
  newOb->setInterpolationWorldTransform(tr);

  newOb->setCollisionShape(m_collisionShapes[1]);


  // m_dynamicsWorld->addCollisionObject(newOb);
#endif

#if 0  // create box object
  btCollisionShape* boxshape = new btBoxShape(btVector3(0.25,0.25,0.25));
  btTransform tr2;
  tr2.setIdentity();
  tr2.setOrigin(btVector3(0, 0, 3.0));
  btRigidBody* boxBody = createRigidBody(41.0, tr2, boxshape);
  int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
  int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  m_dynamicsWorld->addRigidBody(boxBody, collisionFilterGroup, collisionFilterMask);
  btScalar Margin = boxBody->getCollisionShape()->getMargin();
  b3Printf("Margin = %f\n", Margin);
#endif


#ifdef CREATE_TETRACUBE  // create TetraCube object
  btVector3 p1 = btVector3(0, 0, 0.7);
  btVector3 s1 = btVector3(1.90, 1, 0.2);
  TetraCube(this, p1, s1);
#endif

#ifdef CREATE_SOFTBOX  // create softbox object
  btVector3 p2 = btVector3(0, 0, 2);
  btVector3 s2 = btVector3(1, 1, 1);
  Ctor_SoftBox(this, p2, s2);
#endif	  

#ifdef CREATE_CLOTH  // create Cloth Object
  // btScalar s3 = 1.0;
  Cloth(this);
#endif

#ifdef CREATE_MULTIBODY_SPHERE  // create MultiBody Sphere
  btCollisionShape* childShape = new btSphereShape(btScalar(0.06));
  childShape->setMargin(0.1);
  // btCollisionShape* childShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));
  // m_guiHelper->createCollisionShapeGraphicsObject(childShape);

  btScalar mass = 10.f;
  btVector3 baseInertiaDiag;
  bool isFixed = (mass == 0.0);
  childShape->calculateLocalInertia(mass, baseInertiaDiag);
  btMultiBody* pMultiBody = new btMultiBody(0, mass, baseInertiaDiag, false, false);
  btTransform startTrans;
  startTrans.setIdentity();
  startTrans.setOrigin(btVector3(0, 0, 0.8));
  // startTrans.setOrigin(btVector3(0, 0, 1));

  pMultiBody->setBaseWorldTransform(startTrans);

  btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
  col->setCollisionShape(childShape);
  pMultiBody->setBaseCollider(col);
  bool isDynamic = (mass > 0 && !isFixed);
  int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
  int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

  // b3Printf("CollisionGroup = %d\n", collisionFilterGroup);
  // b3Printf("CollisionMask = %d\n", collisionFilterMask);

  m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  
  // m_dynamicsWorld->addCollisionObject(col);  

  pMultiBody->finalizeMultiDof();

  m_dynamicsWorld->addMultiBody(pMultiBody);
#endif

  m_softBodyWorldInfo.m_sparsesdf.Reset();


  m_softBodyWorldInfo.air_density = (btScalar)1.2;
  m_softBodyWorldInfo.water_density = 0;
  m_softBodyWorldInfo.water_offset = 0;
  m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
  m_softBodyWorldInfo.m_gravity.setValue(0, 0, -10);

  m_autocam = false;
  m_raycast = false;
  m_cutting = false;
  m_results.fraction = 1.f;

  // BedFrame::registerModel(this);
  // Mattress::registerModel(this);

#ifdef CREATE_BED_BOX
  // createBedBox(this);
#endif

#ifdef CREATE_COMPOUND_BED
  // createCompoundBed(this);
#endif

#ifdef CREATE_RIGID_SHPERE
  createRigidSphere(this, 20.f);
  // createRigidSphere(this, 0.1);
#endif

#ifdef CREATE_RIGID_BOX
  createRigidBox(this, 1.f);
#endif



#if 0
  createEmptyDynamicsWorld();

  //MuJoCo uses a slightly different collision filter mode, use the FILTER_GROUPAMASKB_OR_GROUPBMASKA2
  //@todo also use the modified collision filter for raycast and other collision related queries
  m_filterCallback->m_filterMode = FILTER_GROUPAMASKB_OR_GROUPBMASKA2;

  //m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);
#endif

  // reading MJCF Models
  if (m_guiHelper->getParameterInterface())
  // if (m_guiHelper->getAppInterface())
  {
    // b3Printf("OK\n");
    SliderParams slider("Gravity", &m_grav);
    slider.m_minVal = -10;
    slider.m_maxVal = 10;
    // Gravityのスライダーを設定
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
  }

  int flags = 0;
  b3BulletDefaultFileIO fileIO;
  BulletMJCFImporter importer(m_guiHelper, 0, &fileIO, flags);
  MyMJCFLogger logger;
  bool result = importer.loadMJCF(m_fileName, &logger);
#ifdef DEBUG_HUMANOID
  if (result)
  {
    btTransform rootTrans;
    rootTrans.setIdentity();

    for (int m = 0; m < 1; m++) // チェック柄の床のみ生成
    // for (int m = 0; m < importer.getNumModels(); m++)
    {

      // 設定した人数より多くの人体モデルを読み込んでいる時、break
      if(num_humanoid < m)
	break;

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

      // change humanoid init orientation and position using KeyPoint-File and RootFile 
      if( !(m == 0) )
      {
        judgeOrientation(m_data, vstr, m-1); 
	b3Printf("  Humanoid %d's Orientation : %s\n", m-1, m_data->m_orientation ? "True" : "False");
	btScalar Bed_edge = 0.8f;
	// baseline内で後ろ向きならhumanoidを180°回転させる
	/*
	if(m_data->m_orientation)
	{
	  // btScalar root_x = 200.f/1000.f; // .fを付けないと小数点以下が切り捨てられる
	  // btScalar temp_y = 300;
	  // btScalar root_y = -temp_y/1000;
	  rootTrans.setOrigin(btVector3(0.f, -Bed_edge, -0.2));
	  btQuaternion q; q.setRotation(btVector3(0, 0, 1), (SIMD_PI*3)/2); // ベッドに背を向けている 
	  rootTrans.setRotation(q);
	}
	else
	*/
	{
	  // btScalar root_x = -200/1000.f;
	  // btScalar temp_y = 250.f;
	  // btScalar root_y = -temp_y/1000.f;
	  // root_z = 0.f;
	  // btQuaternion q; q.setRotation(btVector3(0, 0, 1), SIMD_PI/2); // ベッドの方を向いている 
	  // btQuaternion q; q.setRotation(btVector3(0, 0, 1), (SIMD_PI*3)/2); // ベッドに背を向けている 
	  // btQuaternion q; q.setRotation(btVector3(0, 0, 1), 0); // ベッドは縦向きで、カメラの方向に顔を向けている 
	  // rootTrans.setRotation(q);


	  // b3Printf("   root_x => %f\n", root_x);
	  // b3Printf("   root_y => %f\n", root_y);
	  // b3Printf("   root_z => %f\n", root_z);
	  
	  if( m == 1 )
	  {
	    btScalar root_joint_pos_humanA[3] = {0.f};
	    getInitRootJointPos(root_joint_pos_humanA, 0);
	    btScalar root_x = root_joint_pos_humanA[1];
	    btScalar root_y = btFabs(root_joint_pos_humanA[0]);
	    b3Printf("root_y ======> %f\n", root_y);
	    root_y = root_y / 1000.f;
	    b3Printf("root_x ======> %f\n", root_x);
	    // btScalar root_z = root_joint_pos[2] / 250.f;
      	    rootTrans.setIdentity();
	    btQuaternion q; 
	    rootTrans.setOrigin(btVector3(-1.f, 0.f, 0.85));
	    rootTrans.setOrigin(btVector3(-1.f, -root_y, 0.85)); 
	    rootTrans.setOrigin(btVector3(0.f, 0.f, 10000.f)); 
	    // rootTrans.setOrigin(btVector3(0.f, -1.34f, 0.85)); // IMG1268
	    // q.setRotation(btVector3(0, -1, 0), SIMD_PI/2); // 仰向け(頭手前側)
	    // q.setEulerZYX(3*SIMD_PI/2, -SIMD_PI/2, 0); // 仰向け(頭左側)
	    q.setEulerZYX(SIMD_PI + 20*SIMD_PI/180, -SIMD_PI/2, 0); // 仰向け(頭奥側)
	    root_y = 0.8;
	    // rootTrans.setOrigin(btVector3(0.1f, -root_y, 0.1));
	    // q.setRotation(btVector3(0, 0, 1), -SIMD_PI/2); // ベットに背を向けている 
       	    rootTrans.setRotation(q);
	  }
	  else if( m == 2 )
	  {
	    btScalar root_joint_pos_humanB[3] = {0.f};
	    getInitRootJointPos(root_joint_pos_humanB, 1);
	    btScalar root_y = btFabs(root_joint_pos_humanB[0]);
	    b3Printf("root_y ======> %f\n", root_y);
	    root_y = 1.1;
            rootTrans.setIdentity();
	    rootTrans.setOrigin(btVector3(-0.5f, -root_y, 0.3));
	    rootTrans.setOrigin(btVector3(0.f, 0.f, 100000.f));
	    btQuaternion q; 
	    q.setRotation(btVector3(0, 0, 1), (SIMD_PI/2 - 27*SIMD_PI/180)); // ベッドの方を向いている 
	    q.setEulerZYX(SIMD_PI/2 - 27*SIMD_PI/180, 28*SIMD_PI/180, 0); // 仰向け(頭奥側)
       	    rootTrans.setRotation(q);
	  }
	}
      }



#if 0 // test用（humanoidがBed上に横たわっている）
      if(!m == 0){
        // change humanoid1 start position
        rootTrans.setOrigin(btVector3(1, 0, 0.9)); // BedFrameなしの時
	btQuaternion q; q.setRotation(btVector3(0, -1, 0), SIMD_PI/2); // 仰向け 
	rootTrans.setRotation(q);
      }
      else if(m == 2){
        // change humanoid2 start position
        rootTrans.setOrigin(btVector3(1, -2, -0.2));
      }
      else{
	// set floor position to root
        importer.getRootTransformInWorld(rootTrans);
      }
#endif

      ConvertURDF2Bullet(importer, creation, rootTrans, m_dynamicsWorld, m_useMultiBody, importer.getPathPrefix(), CUF_USE_MJCF);

      mb = creation.getBulletMultiBody();

      // calc humanoid total mass
      btScalar TotalMass = mb->getBaseMass();
      // b3Printf("BaseMass = %d\n", TotalMass);

	if( m == 0 )
	{
          btCollisionObject* ground_col = btMultiBodyLinkCollider::upcast(mb->getBaseCollider());
	  // ground_col->setFriction(1000.0);
          // ground_col->setRollingFriction(1000.0);
	  // b3Printf("ground_friction = %d\n", ground_col->getFriction());
	  // b3Printf("ground_rollingfriction = %d\n", ground_col->getRollingFriction());
	}

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

	m_data->m_numMotors = 0;

        btCollisionObject* base_col = btMultiBodyLinkCollider::upcast(mb->getBaseCollider());
	// base_col->setFriction(2.0);
        // base_col->setRollingFriction(2.0);

	// b3Printf("numLinks = %d", numLinks);


	// 0フレーム目の姿勢を初期姿勢にする
	/*
	if( m == 1 )
	  calcJointAngle(humanA_angle_array, 0, m-1); 
	else if( m == 2 )
	  calcJointAngle(humanB_angle_array, 0, m-1); 
	*/


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
	  // m_data->m_humanoids.push_back(mb);
	  // m_data->m_numHumanoids++;

          TotalMass += mb->getLinkMass(i);

	  // b3Printf("%s\n", linkName->c_str());
	  // b3Printf("	%s\n", jointName->c_str());


          btCollisionObject* link_col = btMultiBodyLinkCollider::upcast(mb->getLinkCollider(i));
	  const btVector3 RGB = btVector3(0, 0, 0);
	  link_col->setCustomDebugColor(RGB);
          // link_col->setCcdSweptSphereRadius(0.00025);
          // link_col->setCcdMotionThreshold(0.001);
	  btCollisionShape* col_shape = link_col->getCollisionShape();

	  link_col->setCollisionFlags(link_col->getCollisionFlags() | btCollisionObject::CF_CHARACTER_OBJECT);

	  link_col->setFriction(1.0);
	  link_col->setRollingFriction(1.0);
	  // b3Printf("friction = %d\n", link_col->getFriction());
	  // b3Printf("rollingfriction = %d\n", link_col->getRollingFriction());

	  /*
	  if(col_shape->isConvex())
	  {
	    b3Printf("%s is sphere shape\n", linkName->c_str());
	    btSphereShape* shape = (btSphereShape*)col_shape;
	    if(shape->getRadius() == 0.04) // 0.02
	    {
	      b3Printf("run\n");
              col_shape->setMargin(0.5);
	    }
	  }
	  */
	  
	  if(*linkName == "left_lower_arm" || *linkName == "link1_33")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 0, 0, 1));
	    col_shape->setMargin(1.);
	    // link_col->setCcdSweptSphereRadius(0.5);
            // link_col->setCcdMotionThreshold(0.0001);

	  }
	  if(*linkName == "right_lower_arm" || *linkName == "link1_28")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 1, 0, 1));
	    col_shape->setMargin(1.);
	    // btScalar margin = col_shape->getMargin();
	    // b3Printf("margin = %f\n", margin);
	    // link_col->setCcdSweptSphereRadius(0.5);
            // link_col->setCcdMotionThreshold(0.0001);

	  }
	  if(*linkName == "left_upper_arm" || *linkName == "link1_30" || *linkName == "link1_31")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 0, 0, 1));
	    // col_shape->setMargin(1.);
	  }
	  if(*linkName == "right_upper_arm" || *linkName == "link1_25" || *linkName == "link1_26")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 0, 0, 1));
	    // col_shape->setMargin(1.);
	  }
	  if(*linkName == "left_foot")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 0, 0, 1));
	    // col_shape->setMargin(1.);
            // link_col->setCollisionFlags(link_col->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	  }
	  if(*linkName == "right_foot")
	  {
	    m_guiHelper->createCollisionShapeGraphicsObject(col_shape);
	    // m_guiHelper->createCollisionObjectGraphicsObject(link_col, btVector4(1, 0, 0, 1));
	    // col_shape->setMargin(1.);
	  }
	  if(*linkName == "pelvis")
	  {
	    // link_col->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
	  }
	    
	  if (mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::eRevolute || mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::ePrismatic)
	  {
	    if (m_data->m_numMotors < MAX_NUM_MOTORS)
	    {
	      char motorName[1024];
	      sprintf(motorName, "%s q ", jointName->c_str());
	      MotorJointNames.push_back(*jointName);
	      btScalar* motorPos = &m_data->m_motorTargetPositions[m_data->m_numMotors];


	      *motorPos = 0.f;
	      // SliderParams slider(motorName, motorPos);
	      // slider.m_minVal = -4;
	      // slider.m_maxVal = 4;
	      // slider.m_clampToIntegers = false;
	      // slider.m_clampToNotches = false;
	      // b3Printf("param = %f\n", m_guiHelper->getParameterInterface());
	      // JointMotorのスライダーを設定
	      // m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	      float maxMotorImpulse = 5.f;
	      btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb, mbLinkIndex, 0, 0, maxMotorImpulse);
	      motor->setErp(0.1);
	      // motor->setMaxAppliedImpulse(0);
	      
	      /*
	      if( m == 1 )
	      {
		btScalar pos = humanA_angle_array[m_data->m_numMotors];
		motor->setPositionTarget(pos);
	      }
	      if( m == 2 )
	      {
		btScalar pos = humanB_angle_array[m_data->m_numMotors];
		motor->setPositionTarget(pos);
	      }
	      */

	      m_data->m_jointMotors[m_data->m_numMotors] = motor;
	      m_dynamicsWorld->addMultiBodyConstraint(motor);
	      m_data->m_numMotors++;
	    }
	  }

          
#if 1
	  // change link color
	  int bodyId = -1;
	  int linkIndex = i;
	  int shapeIndex = i;
          btCollisionObject* col = btMultiBodyLinkCollider::upcast(mb->getLinkCollider(i));
	  UserId++; 
          // col->setUserIndex(UserId);
	  // b3Printf("%d\n", UserId); // default -1
          const double color[4] = { 0, 0, 0, 1 };
	  // m_guiHelper->changeRGBAColor(UserId, color);
#endif
        }
      }
      if(m == 1)
      {
        // b3Printf("TotalMass = %f\n", TotalMass);
      }

      if(m != 0)
      {
	// b3Printf("m_data->m_numMotors = %d\n", m_data->m_numMotors);
        m_datas.push_back(m_data);
	m_data++;
      }
    }
  }
#endif

#if 0
  int texWidth = 1024;
  int texHeight = 1024;
  unsigned char texels = 255;
  m_guiHelper->changeTexture(1, &texels, texWidth, texHeight);
#endif


  
  /*
  // 0フレーム目の姿勢を取らせる
  for(int n = 0; n < m_datas.size(); n++)
  {
    b3Printf("    change Init Pos    \n");

    struct ImportMJCFInternalData* humanoid_data = m_datas[n];

    if( n == 0 )
      calcJointAngle(humanA_angle_array, 0, n); 
    else
      calcJointAngle(humanB_angle_array, 0, n); 
	
    for (int i = 0; i < humanoid_data->m_numMotors; i++)
    {
      if (humanoid_data->m_jointMotors[i])
      {
	btScalar pos;

	if( n == 0 )
	{
          pos = humanA_angle_array[i];
	  // pos = M_PI/6;
	}
	else
          pos = humanB_angle_array[i];

	humanoid_data->m_jointMotors[i]->setPositionTarget(pos);    
      }
    }
  }
  */
  


  btVector4 rgba = btVector4(1, 1, 1, 1);
  Nursing::autogenerateGraphicsObjects(m_dynamicsWorld, rgba);
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



// step Simulation
void Nursing::stepSimulation(float deltaTime)
{
  if (m_dynamicsWorld)
  {
    btVector3 gravity(0, 0, -10);
    gravity[m_upAxis] = m_grav;
    m_dynamicsWorld->setGravity(gravity);

    btScalar fixedTimeStep = 1. / 240.f;
    // btScalar fixedTimeStep = 1. / 120.f;
    
      DrawContactForce();
    //the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
    m_dynamicsWorld->stepSimulation(deltaTime, 10, fixedTimeStep);
  }
}
// ----------------------------------------------------------------------------------

void Nursing::DrawContactForce(btScalar fixedTimeStep)
{ // 赤(255, 0, 0)
  int num_manifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds(); // 衝突候補のペアの数
  for(int i = 0; i < num_manifolds; i++)  // 各ペアを調べていく
  {
    // 衝突点を格納するためのキャッシュ(manifold)から情報を取得
    btPersistentManifold* manifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
#if 1
    btCollisionObject* obA = const_cast<btCollisionObject*>(manifold->getBody0()); // 衝突ペアのうちのオブジェクトA
    btCollisionObject* obB = const_cast<btCollisionObject*>(manifold->getBody1()); // 衝突ペアのうちのオブジェクトB
#endif
#if 0
    // 各オブジェクトのユーザーインデックス
    int user_idx0 = obA->getUserIndex();
    int user_idx1 = obB->getUserIndex();
    b3Printf("user_idx0 = %d\n", user_idx0);
    b3Printf("user_idx1 = %d\n", user_idx1);
#endif

    int num_contacts = manifold->getNumContacts(); // オブジェクト間の衝突点数
    btScalar pointSize = 30;
    int flag = btCollisionObject::CF_KINEMATIC_OBJECT; // Kinematicなobjectとの衝突点は描画しない
    // int flag = btCollisionObject::CF_STATIC_OBJECT; // staticなobjectとの衝突点は描画しない
    int obj_flag = obA->getCollisionFlags() | obB->getCollisionFlags();
    if(!(obj_flag & flag))
    {
      for(int j = 0; j < num_contacts; ++j)
      {
	btManifoldPoint& pt = manifold->getContactPoint(j); // 衝突点キャッシュから衝突点座標を取得
	if(pt.getDistance() <= 0.0f)  // 衝突点間の距離がゼロ以下なら実際に衝突している
	{
	  const btVector3& ptA = pt.getPositionWorldOnA();
	  const btVector3& ptB = pt.getPositionWorldOnB();
	  btScalar contact_impulse = pt.getAppliedImpulse();
	  btScalar contact_force = contact_impulse / fixedTimeStep;
	  b3Printf("contact force = %f\n", contact_force); 
	  // b3Printf("contact point = %f, %f, %f\n", ptA.getX(), ptA.getY(), ptA.getZ());   
	  // b3Printf("contact pointA = %f, %f, %f\n", ptA.getX(), ptA.getY(), ptA.getZ());       
	  // b3Printf("contact pointB = %f, %f, %f\n", ptB.getX(), ptB.getY(), ptB.getZ());       
	  // btScalar step = 10; // デジタル的方法：10[N]ごとに赤色に近づく(Maxは500[N])

	  btScalar delta_color = contact_force / MAX_CONTACT_FORCE;
	  // btVector3 color2 = btVector3(1, 1-delta_color, 1-delta_color);
	  // btVector3 color = white;
	  btVector3 color = green;
	  if( 0.f < contact_force && contact_force < MAX_CONTACT_FORCE/3)
	    color = green;
	  else if( MAX_CONTACT_FORCE/3 < contact_force && contact_force < 2*MAX_CONTACT_FORCE/3)
             color = yellow;
	  else if( 2*MAX_CONTACT_FORCE/3 < contact_force && contact_force < MAX_CONTACT_FORCE)
            color = red;
	  else if( MAX_CONTACT_FORCE < contact_force)
	  {
            color = black;
	    // b3Printf("contact force = %f, %f(imp*dt)\n", contact_force, contact_force); 
	  }
	    
	  m_guiHelper->getRenderInterface()->drawPoint(ptA, color, pointSize);
	}
      }
    }
  }
}


class CommonExampleInterface* NursingCreateFunc(struct CommonExampleOptions& options)
{
  current_demo = options.m_option;
  return new Nursing(options.m_guiHelper);
}
