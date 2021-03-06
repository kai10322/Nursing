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

#define SIMULATION

#define JOINT_ANGLE_SHOW

#define DEBUG_HUMANOID 
#define CREATE_COMPOUND_BED
#define CREATE_CLOTH
// #define CREATE_BED_BOX
// #define CREATE_MULTIBODY_SPHERE
#define CREATE_RIGID_SHPERE
// #define CREATE_RIGID_BOX
// #define CREATE_SOFTBOX
// #define CREATE_TETRACUBE 
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


  // humanoidを何体生成するかを入力（画像内の人の数）
  b3Printf("  Please Input Humanoid Num -> ");
  std::string num;
  getline(std::cin, num);
  num_humanoid = atoi(num.c_str());

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
	  if(num_humanoid == 2)
   	    gMCFJFileNameArray.push_back("./humanoid_model/original_humanoid_2.xml");
	  else 
   	    gMCFJFileNameArray.push_back("./humanoid_model/original_humanoid.xml");
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
  body->setFriction(1);
  return (body);
}


//
// Rigid Box 
//
static btRigidBody* createRigidBox(Nursing* pdemo, btScalar mass = 1.0, btScalar edge_length = 0.5)
{
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, 1.0));
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
	btScalar margin = 0.05; // 衝突マージン // 0.05 or 0.1
        btScalar Bed_height = 0.5*0.38*2 + margin + 0.2; // Bed Height : 0.24*2, Ground Height : 0.2
	btScalar h = 1.2;
        btScalar w = 0.7;
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(
			pdemo->m_softBodyWorldInfo, 
			btVector3(-h*1., -w*1., 0), btVector3(+h*1., -w*1., 0),
                        btVector3(-h*1., +w*1., 0),btVector3(+h*1., +w*1., 0),
                        61, 61, // 17,17, // 51, 51 // 62, 62
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
  // m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
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


  m_dynamicsWorld->addCollisionObject(newOb);
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
  createBedBox(this);
#endif

#ifdef CREATE_COMPOUND_BED
  createCompoundBed(this);
#endif

#ifdef CREATE_RIGID_SHPERE
  createRigidSphere(this, 2.f);
  // createRigidSphere(this, 0.1);
#endif

#ifdef CREATE_RIGID_BOX
  createRigidBox(this, 1.f);
#endif

  // 最初に全フレームのkeypointデータを読み込む
  const std::string dir_path = "./keypoint_folder/";
  std::string filename;
  b3Printf("  Please Input FileName -> ");
  getline(std::cin, filename);
  b3Printf(" ----- Start KeyPoint File Reading ----- \n");
  readKeyPointFile((dir_path+filename).c_str(), vstr);
  b3Printf(" ----- Finish KeyPoint File Reading ----- \n");
  b3Printf("  Total Frame : %d\n", vstr.size()/num_humanoid);
  b3Printf("  Humanoid Num : %d\n", num_humanoid);

  // arrangeKeypointData("./keypoint_folder/arrange_KeypointFile.txt");
  arrangeKeypointData();

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

    // for (int m = 0; m < importer.getNumModels()-1; m++)
    for (int m = 0; m < importer.getNumModels(); m++)
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
	    rootTrans.setOrigin(btVector3(0.f, 0.f, -0.2f)); // 論文用
	    rootTrans.setOrigin(btVector3(1000.f, 0.f, -0.2f)); // 論文用
	    // rootTrans.setOrigin(btVector3(0.f, -1.34f, 0.85)); // IMG1268
	    // q.setRotation(btVector3(0, -1, 0), SIMD_PI/2); // 仰向け(頭手前側)
	    // q.setEulerZYX(3*SIMD_PI/2, -SIMD_PI/2, 0); // 仰向け(頭左側)
	    // q.setEulerZYX(SIMD_PI + 20*SIMD_PI/180, -SIMD_PI/2, 0); // 仰向け(頭奥側)
	    root_y = 0.8;
	    // rootTrans.setOrigin(btVector3(0.1f, -root_y, 0.1));
	    // q.setRotation(btVector3(0, 0, 1), -SIMD_PI/2); // ベットに背を向けている 
       	    // rootTrans.setRotation(q);
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
	  ground_col->setFriction(1.0);
          ground_col->setRollingFriction(1.0);
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



//
// 3x3の行列と3次元ベクトルの積を計算する関数
//
btVector3 calc_rot_mat(btMatrix3x3 rot, btVector3 base)
{
  btVector3 result;
  for(int r = 0; r < 3; r++)
  {
    btVector3 row = rot.getRow(r);
    if(r == 0)
      result.setX(row.dot(base));
    if(r == 1)
      result.setY(row.dot(base));
    if(r == 2)
      result.setZ(row.dot(base));
  }
  return result;
}

//
// 骨格の長さを計算する関数
//
btScalar calc_born_length(btScalar x, btScalar y, btScalar z)
{
  btVector3 born = btVector3(x, y, z);
  return born.length();
}

//
// 身体が正面を向いているか後ろ向きかを判定
// 0 Frame目での、各人のright_hipとleft_hipの位置関係を使う
//
void Nursing::judgeOrientation(struct ImportMJCFInternalData* m_data, const std::vector<std::string> &vstr, int n)
{
  const int joint_num = 17;
  // baselineの骨格モデルを構成しているkeypoint
  unsigned int baseline_joint[joint_num] = {0, 1, 2, 3, 6, 7, 8, 12, 13, 14, 15, 17, 18, 19, 25, 26, 27};   
  // baselineのデータを読み込む配列
  btScalar tmp_pos_data[32][3] = {0.f};

  std::string line_data = vstr[n];

  // カンマ区切りごとにデータを読み込むためにistringstream型にする
  std::istringstream i_stream(line_data);
  std::string tmp;

  unsigned int i = 0, j = 0;
  while(getline(i_stream, tmp, ',')) {
    if(j == 3){
      j = 0;
      i++;
      if(i == 32)
        break;
    }  
  // 文字型のデータをdouble型に変換し、pos_data配列に格納していく
    tmp_pos_data[i][j] = std::stof(tmp);
    j++;
  }

  btScalar pos_data[joint_num][3] = {0.f};

  for (int i = 0; i < 17; i++){
    for(int j = 0; j < 3; j++){
      pos_data[i][j] = tmp_pos_data[baseline_joint[i]][j];
    }
  }

  btScalar right_hip_x = pos_data[1][0];
  btScalar left_hip_x = pos_data[4][0];

  if(left_hip_x < right_hip_x)
    m_data->m_orientation = true;
  else
    m_data->m_orientation = false;
}

//
// 計算対象の骨格が3d-pose-baseline内でちゃんと推定されていたかを判定
// Nursing_keypoint_1.txtの時、500.f
//
bool judgeCalc(const btScalar pos[][3], int a, int b, int c)
{
  btScalar x, y, z;

#if 0
  x = pos[a][0] - pos[b][0];
  y = pos[a][1] - pos[b][1];
  z = pos[a][2] - pos[b][2];
  // if(calc_born_length(x, y, z) > 1500.f)
  if(calc_born_length(x, y, z) > 500.f)
  {
    return false;
  }

  x = pos[c][0] - pos[b][0];
  y = pos[c][1] - pos[b][1];
  z = pos[c][2] - pos[b][2];
  // if(calc_born_length(x, y, z) > 1500.f)
  if(calc_born_length(x, y, z) > 500.f)
  {
    return false;
  }
#endif

  if(pos[a][0] == 0.f && pos[a][1] == 0.f && pos[a][2] == 0.f &&
     pos[b][0] == 0.f && pos[b][1] == 0.f && pos[b][2] == 0.f &&
     pos[c][0] == 0.f && pos[c][1] == 0.f && pos[c][2] == 0.f
     )
    return false;


  return true;
}

// 1行ずつデータを読み込み、vectorに格納
void Nursing::readKeyPointFile(const char* filename, std::vector<std::string> &vstr)
{
  std::ifstream pos_file(filename);

  if(!pos_file) {
    b3Printf(" Cannot Open Keypoint File!!!\n");
    exit(1);
  }
  else
    b3Printf("  reading keypoint file : %s\n", filename);

  std::string line_data;
  while(getline(pos_file, line_data))
  {
    line_data.erase(line_data.find('['), 1);
    line_data.erase(line_data.find(']'), 1);
    vstr.push_back(line_data);
  }

  return;
}

//
// keypointの検出されている順番を、同じ順番になるように並び替える
//
// void Nursing::arrangeKeypointData(const char* output_filename)
void Nursing::arrangeKeypointData()
{
  b3Printf(" -----  arrange keypoint data----- \n");

  // std::ofstream output_file(output_filename);
  // output_file.open(output_filename, std::ios::out);


  // const int joint_num = 17;
  // baselineの骨格モデルを構成しているkeypoint
  // unsigned int baseline_joint[joint_num] = {0, 1, 2, 3, 6, 7, 8, 12, 13, 14, 15, 17, 18, 19, 25, 26, 27};   
  // baselineのデータを読み込む配列
  // btScalar tmp_pos_data[32][3] = {0.f};

  btScalar humanA_root_x = 0.f;
  btScalar humanA_before_root_x = 0.f;
  btScalar humanB_root_x = 0.f;
  btScalar humanB_before_root_x = 0.f;

  // x座標があまりにも急に変化している時、データを入れ替える
  for(int data_num = 0; data_num < vstr.size(); data_num++) // 1フレーム目にrootが検出されていないときも考慮
  {
    if(humanA_root_x != 0.f && humanB_root_x !=0.f)
      break;

    std::string line_data = vstr[data_num];

    std::istringstream i_stream(line_data);
    std::string tmp;

    unsigned int i = 0, j = 0;
    while(getline(i_stream, tmp, ',')) {
      if( i == 8 && j == 0 )
      {
	// thoraxを基準にする
	if(std::stof(tmp) != 0.f && data_num%2 == 0)
	{
	  humanA_root_x = std::stof(tmp);
	}
	if(std::stof(tmp) != 0.f && data_num%2 != 0)
	{
	  humanB_root_x = std::stof(tmp);
	}
	break;
      }
    }
    j++;
  }

  for(int data_num = 2; data_num < vstr.size()-1; data_num++)
  {
    std::string line_data_1 = vstr[data_num];
    std::string line_data_2 = vstr[data_num + 1];

    std::istringstream i_stream(line_data_1);
    std::string tmp;

    btScalar root_x = 0.f;
    unsigned int i = 0, j = 0;
    while(getline(i_stream, tmp, ',')) {
      if( i == 8 && j == 0 )
      {
	// humanAのthoraxが推定されている時
	if( std::stof(tmp) != 0.f && data_num%2 == 0 )
	{
	  root_x = std::stof(tmp);
	  if(btFabs(humanA_root_x - root_x) > 100.f)
	  {
	    vstr[data_num] = line_data_2;
	    vstr[data_num + 1] = line_data_1;
	    b3Printf("arrange frame = %d\n", data_num/2);
	  }
	  humanA_before_root_x = humanA_root_x;
	  humanA_root_x = root_x;

	  break;
	}
	// humanBのthoraxが推定されている時
	if( std::stof(tmp) != 0.f && data_num%2 != 0 )
	{
	  root_x = std::stof(tmp);
	  if(btFabs(humanB_root_x - root_x) > 100.f)
	  {
	    vstr[data_num - 1] = line_data_2;
	    vstr[data_num] = line_data_1;
	    b3Printf("arrange frame = %d\n", data_num/2);
	  }
	  humanB_before_root_x = humanB_root_x;
	  humanB_root_x = root_x;

	  break;
	}
      }
      j++;
    }
  }

  /*
  for(int data_num = 0; data_num < vstr.size(); data_num++)
  {
    std::string line_data = vstr[data_num];
    output_file << "[" << line_data << "]" << std::endl;
  }

  output_file.close();
  */
}


//
// fフレーム目のn番目の人のデータを取得
//
void Nursing::getPosData(const std::vector<std::string> &vstr, btScalar pos_data[][3], int frame, int num)
{
  b3Printf(" ----- reading frame : %d  human : %d  keypoint data ----- \n", frame, num);

  const int joint_num = 17;
  // keypoint座標を格納する配列
  // btScalar pos_data[joint_num][3] = {0.f};
  // baselineの骨格モデルを構成しているkeypoint
  unsigned int baseline_joint[joint_num] = {0, 1, 2, 3, 6, 7, 8, 12, 13, 14, 15, 17, 18, 19, 25, 26, 27};   
  // baselineのデータを読み込む配列
  btScalar tmp_pos_data[32][3] = {0.f};

  std::string line_data = vstr[frame*num_humanoid + num];

  // カンマ区切りごとにデータを読み込むためにistringstream型にする
  std::istringstream i_stream(line_data);
  std::string tmp;

  unsigned int i = 0, j = 0;
  while(getline(i_stream, tmp, ',')) {
    if(j == 3){
      j = 0;
      i++;
      if(i == 32)
        break;
    }  
  // 文字型のデータをdouble型に変換し、pos_data配列に格納していく
  tmp_pos_data[i][j] = std::stof(tmp);
  // b3Printf("tmp_pos_data[%d][%d] : %f \n", i, j, tmp_pos_data[i][j]);     // pos_data = {{x1,y1,z1},{x2,y2,z2},……}
    j++;
  }

  for (int i = 0; i < 17; i++){
    for(int j = 0; j < 3; j++){
      pos_data[i][j] = tmp_pos_data[baseline_joint[i]][j];
      // b3Printf("pos_data[%d][%d] : %f \n", i, j, pos_data[i][j]); 
    }
  }
}

void Nursing::getInitRootJointPos(btScalar *root_joint_pos, int num)
{
  btScalar pos_data[17][3] = {0.f};
  int frame = 0; 
  getPosData(vstr, pos_data, frame, num);
  
  root_joint_pos[0] = pos_data[0][0]; // x
  root_joint_pos[1] = pos_data[0][1]; // y
  root_joint_pos[2] = pos_data[0][2]; // z
}

//
// 各Jointの角度を計算し、引数のangle_arrayに格納する
//
void Nursing::calcJointAngle(btScalar *angle_array, int frame, int num)
{
  btScalar pos_data[17][3] = {0.f};
  getPosData(vstr, pos_data, frame, num);

  b3Printf(" ----- calc frame : %d human : %d joint Angle ----- \n", frame, num);


  // 変数定義
  // 座標軸はbaselineの座標軸を基にしている
  /*
  btScalar a_x = 0.f; btScalar a_y = 0.f; btScalar a_z = 0.f;
  btVector3 a = btVector3(0.f, 0.f, 0.f);
  btScalar b_x = 0.f; btScalar b_y = 0.f; btScalar b_z = 0.f;
  btVector3 b = btVector3(0.f, 0.f, 0.f);
  btScalar a2_x = 0.f; btScalar a2_y = 0.f; btScalar a2_z = 0.f;
  btVector3 a2 = btVector3(0.f, 0.f, 0.f);
  btScalar b2_x = 0.f; btScalar b2_y = 0.f; btScalar b2_z = 0.f;
  btVector3 b2 = btVector3(0.f, 0.f, 0.f);
  btVector3 rot_a = btVector3(0.f, 0.f, 0.f); // 
  btVector3 rot_b = btVector3(0.f, 0.f, 0.f);
  */
  btVector3 plane_crossVec = btVector3(0.f, 0.f, 0.f); // 平面の法線ベクトル
  btVector3 temp_crossVec = btVector3(0.f, 0.f, 0.f); // 回転方向を判定するための法線ベクトル
  btScalar d = 0.f; btScalar N = 0.f; // 内積を計算するための変数
  btScalar norm = 0.f; // ベクトルの長さ
  // btScalar rot_x = 0.f; btScalar rot_y = 0.f; btScalar rot_z = 0;
  btMatrix3x3 rot_mat; // 回転行列
  btMatrix3x3 Rx; // X軸周りの回転行列
  btMatrix3x3 Ry; // Y軸周りの回転行列
  btMatrix3x3 Rz; // Z軸周りの回転行列
  btVector3  rot_test1 = btVector3(0.f, 0.f, 0.f); // テスト用
  btVector3  rot_test2 = btVector3(0.f, 0.f, 0.f); // テスト用
  // btVector3  project_rot_a = btVector3(0.f, 0.f, 0.f);
  // btVector3  negative_Y_axis = btVector3(0, -1, 0);
  // btVector3 plane_rot_a = btVector3(0.f, 0.f, 0.f);
  // btVector3 project_plane_crossVec = btVector3(0.f, 0.f, 0.f);

  // 初期姿勢からの回転角度
  btScalar rotA = 0.f;
  btScalar rotB = 0.f;


  
#if 1 	// baselineのkeypoint : 0,8（脊椎を無視する）を胴体部分とする
  // 
  // calc abdomen joint angle
  //
  b3Printf("    --- calc abdomen joint angle --- \n");
  if( judgeCalc(pos_data, 0, 7, 8) ) 
  {
    btScalar torso_x = pos_data[8][0] - pos_data[0][0];
    btScalar torso_y = pos_data[8][1] - pos_data[0][1];
    btScalar torso_z = pos_data[8][2] - pos_data[0][2];

    if(m_datas[num]->m_orientation)
    {
      // baseline内で後ろ向きに推定されていたら、胴体の向きを変える
      b3Printf("  baseline model is reverse\n ");
      torso_x = -torso_x;
      torso_y = -torso_y;
    }

    btVector3 torso = btVector3(torso_x, torso_y, torso_z);  // 胴体ベクトル

    btVector3 init_torso = btVector3(0.f, 0.f, 1.f); // 初期胴体ベクトル
    btVector3 init_torso_crossVec = btVector3(1.f, 0.f, 0.f); // 初期胴体の法線ベクトル（上半身が前に傾いている時を正とする）

    // 
    // calc abdomen_y angle
    // 胴体をYZ平面に投影したベクトルと初期胴体とのなす角を計算
    btVector3 project_torso = btVector3(0.f, torso.getY(), torso.getZ()); // 投影したベクトル
    d = project_torso.dot(init_torso);
    N = project_torso.norm() * init_torso.norm();

    // YZ平面に投影した胴体と初期胴体との外積を計算
    project_torso = btVector3(0.f, torso.getY(), torso.getZ());
    temp_crossVec = init_torso.cross(project_torso);

    if(temp_crossVec.getX() >= 0.0) 
    {
      // 胴体を前に倒している時
      angle_array[1] = acos(d/N);
      Rx = btMatrix3x3(1, 0, 0,
                       0, btCos(angle_array[1]), -btSin(angle_array[1]),
                       0, btSin(angle_array[1]), btCos(angle_array[1]));
    }
    else
    {
      angle_array[1] = -acos(d/N);
/*
      // baselineの推定結果によっては、身体を前に傾けているはずなのにthoraxのkeypointがpelvisより後ろにあったりするので、これを矯正するための処理
      // 明らかに傾けている時、回転方向を変える
      // しかし、こうすると初期胴体に一致しなくなる
      btScalar temp1 = pos_data[7][0] - pos_data[0][0];
      btScalar temp2 = pos_data[8][0] - pos_data[7][0];
      btVector3 y1 = btVector3(0.f, temp1, 0.f);
      btVector3 y2 = btVector3(0.f, temp2, 0.f);
      btVector3 judge_cross = y2.cross(y1);
      if(judge_cross.getX() >= 0.f)
        angle_array[1] = -angle_array[1];
*/

      Rx = btMatrix3x3(1, 0, 0,
                       0, btCos(angle_array[1]), -btSin(angle_array[1]),
                       0, btSin(angle_array[1]), btCos(angle_array[1]));
    }

#ifdef JOINT_ANGLE_SHOW
    b3Printf("abdomen_y = %f\n", (angle_array[1]*180)/M_PI);
#endif

    //
    // calc abdomen_x angle
    // 胴体と胴体をYZ平面に投影したベクトルとのなす角を計算
    project_torso = btVector3(0.f, torso.getY(), torso.getZ());
    d = torso.dot(project_torso);
    N = torso.norm() * project_torso.norm();

    // 初期胴体とXZ平面に投影した胴体との外積を計算
    project_torso = btVector3(torso.getX(), 0.f, torso.getZ());
    temp_crossVec = init_torso.cross(project_torso);

    if(temp_crossVec.getY() >= 0)
    {
      // 体を左側に倒している時
      angle_array[2] = -acos(d/N);
      Ry = btMatrix3x3(btCos(-angle_array[2]), 0, btSin(-angle_array[2]),
                       0, 1, 0, 
                      -btSin(-angle_array[2]), 0, btCos(-angle_array[2]));
    }
    else
    {
      angle_array[2] = acos(d/N);
      Ry = btMatrix3x3(btCos(-angle_array[2]), 0, btSin(-angle_array[2]),
		       0, 1, 0, 
		      -btSin(-angle_array[2]), 0, btCos(-angle_array[2]));
    }

#ifdef JOINT_ANGLE_SHOW
    b3Printf("abdomen_x = %f\n", (angle_array[2]*180)/M_PI);
#endif

    // 初期胴体に一致するか確認
    rot_mat = Rx*Ry;
    rot_test1 = btVector3(rot_mat.tdotx(torso), rot_mat.tdoty(torso), rot_mat.tdotz(torso));
    // b3Printf("baseline:torso   x =  %f y = %f z = %f\n", torso.getX(), torso.getY(), torso.getZ());
    // b3Printf(" torso -> Init-Toros :: x = %f, y = %f, z = %f\n", rot_test1.getX()/rot_test1.norm(), rot_test1.getY()/rot_test1.norm(), rot_test1.getZ()/rot_test1.norm());

    // 
    // calc abdomen_z angle
    // 胸部平面と下腹部平面の法線ベクトルのなす角をねじれとする
    btScalar a_x = pos_data[1][0] - pos_data[8][0];
    btScalar a_y = pos_data[1][1] - pos_data[8][1];
    btScalar a_z = pos_data[1][2] - pos_data[8][2];
    btVector3 a = btVector3(a_x, a_y, a_z);		// 下腹部平面内の1ベクトル
    btScalar b_x = pos_data[4][0] - pos_data[8][0];
    btScalar b_y = pos_data[4][1] - pos_data[8][1];
    btScalar b_z = pos_data[4][2] - pos_data[8][2];
    btVector3 b = btVector3(b_x, b_y, b_z);		// 下腹部平面内の1ベクトル
    btVector3 waistPlane_crossVec = a.cross(b);	// 下腹部平面の法線ベクトル
    btScalar a2_x = pos_data[11][0] - pos_data[0][0];
    btScalar a2_y = pos_data[11][1] - pos_data[0][1];
    btScalar a2_z = pos_data[11][2] - pos_data[0][2];
    btVector3 a2 = btVector3(a2_x, a2_y, a2_z);		// 胸部平面内の1ベクトル
    btScalar b2_x = pos_data[14][0] - pos_data[0][0];
    btScalar b2_y = pos_data[14][1] - pos_data[0][1];
    btScalar b2_z = pos_data[14][2] - pos_data[0][2];
    btVector3 b2 = btVector3(b2_x, b2_y, b2_z);		// 胸部平面内の1ベクトル
    btVector3 thoraxPlane_crossVec = a2.cross(b2);	// 胸部平面の法線ベクトル

    // b3Printf("waistPlane_CrossVec :: x = %f, y = %f, z = %f\n", waistPlane_crossVec.getX(), waistPlane_crossVec.getY(), waistPlane_crossVec.getZ());
    // b3Printf("thoraxPlane_CrossVec :: x = %f, y = %f, z = %f\n", thoraxPlane_crossVec.getX(), thoraxPlane_crossVec.getY(), thoraxPlane_crossVec.getZ());

    // 胴体矯正後（初期位置に一致させた時）の下腹部と胴体の法線ベクトル
    btVector3 rot_waistPlane_crossVec = btVector3(rot_mat.tdotx(waistPlane_crossVec), rot_mat.tdoty(waistPlane_crossVec), rot_mat.tdotz(waistPlane_crossVec));
    btVector3 rot_thoraxPlane_crossVec = btVector3(rot_mat.tdotx(thoraxPlane_crossVec), rot_mat.tdoty(thoraxPlane_crossVec), rot_mat.tdotz(thoraxPlane_crossVec));
    // btScalar norm = rot_waistPlane_crossVec.norm();
    // b3Printf("rot_waistPlane_CrossVec :: x = %f, y = %f, z = %f\n", rot_waistPlane_crossVec.getX()/norm, rot_waistPlane_crossVec.getY()/norm, rot_waistPlane_crossVec.getZ()/norm);
    // norm = rot_thoraxPlane_crossVec.norm();
    // b3Printf("rot_thoraxPlane_CrossVec :: x = %f, y = %f, z = %f\n", rot_thoraxPlane_crossVec.getX()/norm, rot_thoraxPlane_crossVec.getY()/norm, rot_thoraxPlane_crossVec.getZ()/norm);

    btVector3 project_rot_waistPlane_crossVec = rot_waistPlane_crossVec;
    btVector3 project_rot_thoraxPlane_crossVec = rot_thoraxPlane_crossVec;
    // 矯正後の法線ベクトルをXY平面に投影
    project_rot_waistPlane_crossVec.setZ(0.f);
    project_rot_thoraxPlane_crossVec.setZ(0.f);
    //
    // rot_waistPlane_crossVec = waistPlane_crossVec;
    // rot_thoraxPlane_crossVec = thoraxPlane_crossVec;
    // rot_waistPlane_crossVec.setZ(0.f);
    // rot_thoraxPlane_crossVec.setZ(0.f);



    // なす角を計算
    d = project_rot_thoraxPlane_crossVec.dot(project_rot_waistPlane_crossVec);
    N = project_rot_thoraxPlane_crossVec.norm() * project_rot_waistPlane_crossVec.norm();

    // 外積により、下腹部から見て胸部がどっち向きにねじれているか確かめる
    temp_crossVec = project_rot_waistPlane_crossVec.cross(project_rot_thoraxPlane_crossVec);
    if(temp_crossVec.getZ() >= 0.0) 
    {
      // 胸部が初期位置より反時計回りに拗じられている時
      angle_array[0] = acos(d/N);
      Rz = btMatrix3x3(btCos(angle_array[0]), -btSin(angle_array[0]), 0,
		       btSin(angle_array[0]), btCos(angle_array[0]), 0, 
		       0, 0, 1);
    }
    else
    {
      angle_array[0] = -acos(d/N);
      Rz = btMatrix3x3(btCos(angle_array[0]), -btSin(angle_array[0]), 0,
		       btSin(angle_array[0]), btCos(angle_array[0]), 0, 
		       0, 0, 1);
    }

#ifdef JOINT_ANGLE_SHOW
    b3Printf("abdomen_z = %f\n", (angle_array[0]*180)/M_PI);
#endif

    // Z方向に回転させた時の胸部ベクトルと矯正後の下腹部ベクトルが同一平面上にあるか確認
    // 法線ベクトルが一致しているか確かめる
    // (-Rz*thorax)×waist = project_waist(On XY-Plane)×waistかどうか
    // project_waist(On XY-Plane)×waist
    // rot_test1 = project_rot_waistPlane_crossVec.cross(rot_waistPlane_crossVec);
    // norm = rot_test1.norm();
    // b3Printf("project_waist(On XY-Plane).cross(waist) :: x = %f y = %f z = %f\n", rot_test1.getX()/norm, rot_test1.getY()/norm, rot_test1.getZ()/norm);
    // (-Rz*thorax)×waist
    rot_test1 = btVector3(Rz.tdotx(rot_thoraxPlane_crossVec), Rz.tdoty(rot_thoraxPlane_crossVec), Rz.tdotz(rot_thoraxPlane_crossVec));
    rot_test2 = rot_test1.cross(rot_waistPlane_crossVec);
    norm = rot_test2.norm();
    // b3Printf("(-Rz*thorax).cross(waist) -> XY-Plane):: x = %f y = %f z = %f\n", rot_test2.getX()/norm, rot_test2.getY()/norm, rot_test2.getZ()/norm);
  }
  else
  {
    b3Printf("  Abdomen Is Not Estimated Correctly!\n");
    angle_array[0] = NOT_ESTIMATED;
    angle_array[1] = NOT_ESTIMATED;
    angle_array[2] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1 
  //
  // calc right_hip joint angle
  //
  if( judgeCalc(pos_data, 0, 1, 2) || judgeCalc(pos_data, 4, 1, 2))
  {
    b3Printf("    --- calc right_hip joint angle --- \n");

    const btVector3 init_upper_leg = btVector3(0.f, 0.f, -1.f); // 初期上脚ベクトル
    const btVector3 init_pelvis_vec = btVector3(1.f, 0.f, 0.f); // 初期骨盤ベクトル
    const btVector3 init_pelvis_crossVec = btVector3(0.f, 0.f, 1.f); // 初期骨盤の法線ベクトル(初期骨盤がx軸上に存在しているので、z軸を法線ベクトルとする)

    btScalar pelvis_x = 0.f; btScalar pelvis_y = 0.f; btScalar pelvis_z = 0.f; 

    if(judgeCalc(pos_data, 4, 1, 2))
    {
      // 逆側のhipもちゃんと推定されている時
      pelvis_x = pos_data[4][0] - pos_data[1][0];
      pelvis_y = pos_data[4][1] - pos_data[1][1];
      pelvis_z = pos_data[4][2] - pos_data[1][2];
    }
    else
    {
      // そうでない時は真ん中のhipを使う
      pelvis_x = pos_data[0][0] - pos_data[1][0];
      pelvis_y = pos_data[0][1] - pos_data[1][1];
      pelvis_z = pos_data[0][2] - pos_data[1][2];
    }

    btVector3 pelvis = btVector3(pelvis_x, pelvis_y, pelvis_z);		// 現在の骨盤ベクトル

    // 骨盤が初期位置からどれだけズレて（回転して）いるかを計算
    // 初期骨盤平面の法線ベクトルと現在の骨盤ベクトルのなす角を計算
    //  calc rotA  //
    d = pelvis.dot(init_pelvis_crossVec);
    N = pelvis.norm() * init_pelvis_crossVec.norm();

    btVector3 project_pelvis = btVector3(pelvis.getX(), 0.0, pelvis.getZ());
    temp_crossVec = init_pelvis_vec.cross(project_pelvis);
    if(acos(d/N) <= M_PI/2)
    // if(temp_crossVec.getY() >= 0.0)
    {
      // 現在の骨盤が平面より上側にある時
      rotA = M_PI/2 - acos(d/N);
      Ry = btMatrix3x3(btCos(-rotA), 0, btSin(-rotA),
		       0, 1, 0,
		      -btSin(-rotA), 0, btCos(-rotA));
      // b3Printf("calc rotA (above plane) : rotA = %f\n", (rotA*180)/M_PI);
    }
    else
    {
      // 現在の骨盤が平面より下側にある時
      rotA = acos(d/N) - (M_PI/2);
      Ry = btMatrix3x3(btCos(rotA), 0, btSin(rotA),
		       0, 1, 0,
		      -btSin(rotA), 0, btCos(rotA));
      // b3Printf("calc rotA (under plane) : rotA = %f\n", (rotA*180)/M_PI);
    }

    // 回転後の骨盤ベクトルと初期骨盤ベクトルのなす角を計算
    // 	calc rotB  //
    project_pelvis = btVector3(pelvis.getX(), pelvis.getY(), 0.0);
    d = project_pelvis.dot(init_pelvis_vec);
    N = project_pelvis.norm() * init_pelvis_vec.norm();
    // rotBの回転方向を求める
    temp_crossVec = init_pelvis_vec.cross(project_pelvis);
    if(temp_crossVec.getZ() > 0.0) 
    {
      rotB = acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0,
		       0, 0, 1);
    }
    else
    {
      rotB = -acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0,
		       0, 0, 1);
    }
    // b3Printf("rotB = %f\n", (rotB*180)/M_PI);

    btScalar a_x = pos_data[2][0] - pos_data[1][0];
    btScalar a_y = pos_data[2][1] - pos_data[1][1];
    btScalar a_z = pos_data[2][2] - pos_data[1][2];
    btVector3 a = btVector3(a_x, a_y, a_z);		// 平面内の1ベクトル
    btScalar b_x = pos_data[3][0] - pos_data[2][0];
    btScalar b_y = pos_data[3][1] - pos_data[2][1];
    btScalar b_z = pos_data[3][2] - pos_data[2][2];
    btVector3 b = btVector3(b_x, b_y, b_z);		// 平面内の1ベクトル


    // 上脚，下脚，骨盤を回転させる
    rot_mat = Rz*Ry; // rot_mat : p0 -> p
    btVector3 rot_pelvis = btVector3(rot_mat.tdotx(pelvis), rot_mat.tdoty(pelvis), rot_mat.tdotz(pelvis)); 
    btVector3 rot_a = btVector3(rot_mat.tdotx(a), rot_mat.tdoty(a), rot_mat.tdotz(a)); 
    btVector3 rot_b = btVector3(rot_mat.tdotx(b), rot_mat.tdoty(b), rot_mat.tdotz(b));

    norm = rot_pelvis.norm();
    // b3Printf(" baseline:pelvis -> Init-Pelvis :: x = %f, y = %f, z = %f\n", rot_pelvis.getX()/norm,rot_pelvis.getY()/norm, rot_pelvis.getZ()/norm);

    // 初期脚平面の法線ベクトル(膝の回転軸と同じ)
    btVector3 init_plane_crossVec = btVector3(-1.f, 0.f, 0.f);

    //
    //	calc right_hip_z angle
    // ねじれを計算するため、矯正後の上脚ベクトルをXY平面に投影
    btVector3 project_rot_a = btVector3(rot_a.getX(), rot_a.getY(), 0.0);
    btVector3 plane_rot_a = -rot_a;
    plane_crossVec = plane_rot_a.cross(rot_b);
    btVector3 project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);

    // 初期脚平面の法線ベクトルとxy平面に投影した法線ベクトルとのなす角を計算
    d = init_plane_crossVec.dot(project_plane_crossVec);
    N = init_plane_crossVec.norm() * project_plane_crossVec.norm();
    temp_crossVec = project_plane_crossVec.cross(init_plane_crossVec);

    if(temp_crossVec.getZ() >= 0.0)
    {
      // 上から見て時計回りに脚をねじっている
      angle_array[3] = -acos(d/N);
    }
    else
    {
      angle_array[3] = acos(d/N);
    }
    Rz = btMatrix3x3(btCos(angle_array[3]), -btSin(angle_array[3]), 0,
		     btSin(angle_array[3]), btCos(angle_array[3]), 0,
		     0, 0, 1);

    b3Printf("right_hip_z = %f\n", (angle_array[3]*180)/M_PI);

    // 
    // calc right_hip_x angle
    // XY平面に投影した脚平面の法線ベクトルと法線ベクトルのなす角を計算
    project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);
    d = project_plane_crossVec.dot(plane_crossVec);
    N = project_plane_crossVec.norm() * plane_crossVec.norm();

    // 初期上脚とYZ平面に投影した矯正後の上脚の外積を計算
    project_rot_a = btVector3(rot_a.getX(), 0.f, rot_a.getZ());
    temp_crossVec = project_rot_a.cross(init_upper_leg);

    if(temp_crossVec.getX() >= 0)
    {
      // 股を開いている
      angle_array[4] = -acos(d/N);
    }
    else
    {
      angle_array[4] = acos(d/N);
    }

    Ry = btMatrix3x3(btCos(angle_array[4]), 0, btSin(angle_array[4]),
		     0, 1, 0,
		    -btSin(angle_array[4]), 0, btCos(angle_array[4]));

    b3Printf("right_hip_x = %f\n", (angle_array[4]*180)/M_PI);

    rot_mat = Rz*Ry;

    // 脚平面の法線ベクトルがX軸負方向に一致するか確認
    rot_test1 = btVector3(rot_mat.tdotx(plane_crossVec), rot_mat.tdoty(plane_crossVec), rot_mat.tdotz(plane_crossVec));
    norm = rot_test1.norm();
    // b3Printf(" baseline:leg-plane-crossVec -> Negative-X-Axis :: x = %f, y = %f, z = %f\n", rot_test1.getX()/norm,rot_test1.getY()/norm, rot_test1.getZ()/norm);

    // 
    // calc right_hip_y angle
    // 初期上脚と脚平面の法線ベクトルがX軸負方向に一致している時（XZ平面にある時）のなす角を計算
    btVector3 rot_a2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
    d = init_upper_leg.dot(rot_a2);
    N = init_upper_leg.norm() * rot_a2.norm();

    temp_crossVec = rot_a2.cross(init_upper_leg);
    if(temp_crossVec.getX() >= 0)
    {
      // 振り上げている
      angle_array[5] = -acos(d/N);
    }
    else
    {
      angle_array[5] = acos(d/N);
    }
    Rx = btMatrix3x3(1, 0, 0,
		     0, btCos(angle_array[5]), -btSin(angle_array[5]),
		     0, btSin(angle_array[5]), btCos(angle_array[5]));

    b3Printf("right_hip_y = %f\n", (angle_array[5]*180)/M_PI);

    rot_mat = Rz*Ry*Rx; // Bullet :: Rz*Rx*Ry(回転行列), Z->X->Y(オイラー)

    rot_test1 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
    norm = rot_test1.norm();
    // b3Printf(" baseline:upper_leg -> Init-UpperLeg :: x = %f, y = %f, z = %f\n", rot_test1.getX()/norm,rot_test1.getY()/norm, rot_test1.getZ()/norm);
  }
  else
  {
    b3Printf("  Right Hip Is Not Estimated Correctly!\n");
    angle_array[3] = NOT_ESTIMATED;
    angle_array[4] = NOT_ESTIMATED;
    angle_array[5] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc right_knee joint angle
  //
  if( judgeCalc(pos_data, 1, 2, 3) )
  {
    b3Printf("    --- calc right_knee joint angle --- \n");
    btScalar a_x = pos_data[1][0] - pos_data[2][0];
    btScalar a_y = pos_data[1][1] - pos_data[2][1];
    btScalar a_z = pos_data[1][2] - pos_data[2][2];
    btVector3 a = -btVector3(a_x, a_y, a_z);
    btScalar b_x = pos_data[3][0] - pos_data[2][0];
    btScalar b_y = pos_data[3][1] - pos_data[2][1];
    btScalar b_z = pos_data[3][2] - pos_data[2][2];
    btVector3 b = btVector3(b_x, b_y, b_z);
    d = a.dot(b);
    N = a.norm() * b.norm();
    angle_array[6] = -acos(d/N);
    b3Printf("right_knee = %f\n", (angle_array[6]*180)/M_PI);
  }
  else
  {
    b3Printf("  Right Knee Is Not Estimated Correctly!\n");
    angle_array[6] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc left_hip joint angle
  //
  if( judgeCalc(pos_data, 0, 4, 5) || judgeCalc(pos_data, 1, 4, 5)) 
  {
    b3Printf("    --- calc left_hip joint angle --- \n");
    const btVector3 init_upper_leg = btVector3(0.f, 0.f, -1.f); // 初期上脚ベクトル
    const btVector3 init_pelvis_vec = btVector3(-1.f, 0.f, 0.f); // 初期骨盤ベクトル
    const btVector3 init_pelvis_crossVec = btVector3(0.f, 0.f, 1.f); // 初期骨盤の法線ベクトル(初期骨盤がx軸上に存在しているので、z軸を法線ベクトルとする)

    btScalar pelvis_x = 0.f; btScalar pelvis_y = 0.f; btScalar pelvis_z = 0.f; 

    if( judgeCalc(pos_data, 1, 4, 5) )
    {	
      pelvis_x = pos_data[1][0] - pos_data[4][0];
      pelvis_y = pos_data[1][1] - pos_data[4][1];
      pelvis_z = pos_data[1][2] - pos_data[4][2];
    }
    else
    {
      pelvis_x = pos_data[0][0] - pos_data[4][0];
      pelvis_y = pos_data[0][1] - pos_data[4][1];
      pelvis_z = pos_data[0][2] - pos_data[4][2];
    }
    btVector3 pelvis = btVector3(pelvis_x, pelvis_y, pelvis_z);	// 骨盤ベクトル
    // 初期骨盤平面の法線ベクトルと現在の骨盤ベクトルのなす角を計算
    //  calc rotA  //
    d = pelvis.dot(init_pelvis_crossVec);
    N = pelvis.norm() * init_pelvis_crossVec.norm();

    btVector3 project_pelvis = btVector3(pelvis.getX(), 0.0, pelvis.getZ());
    temp_crossVec = init_pelvis_vec.cross(project_pelvis);
    if(acos(d/N) <= M_PI/2)
    // if(temp_crossVec.getY() >= 0.0)
    {
      // 現在の骨盤が平面より上側にある時
      rotA = M_PI/2 - acos(d/N);
      Ry = btMatrix3x3(btCos(rotA), 0, btSin(rotA),
		       0, 1, 0, 
		      -btSin(rotA), 0, btCos(rotA));
      // b3Printf("calc rotA (above plane) : rotA = %f\n", (rotA*180)/M_PI);
    }
    else
    {
      // 現在の骨盤が平面より下側にある時
      rotA = acos(d/N) - (M_PI/2);
      Ry = btMatrix3x3(btCos(-rotA), 0, btSin(-rotA),
		       0, 1, 0, 
		      -btSin(-rotA), 0, btCos(-rotA));
      // b3Printf("calc rotA (under plane) : rotA = %f\n", (rotA*180)/M_PI);
    }

    // 回転後の骨盤ベクトルと初期骨盤ベクトルのなす角を計算
    // 	calc rotB  //	
    project_pelvis = btVector3(pelvis.getX(), pelvis.getY(), 0.0);
    d = project_pelvis.dot(init_pelvis_vec);
    N = project_pelvis.norm() * init_pelvis_vec.norm();
    // rotBの回転方向を求める
    temp_crossVec = init_pelvis_vec.cross(project_pelvis);
    if(temp_crossVec.getZ() > 0.0) 
    {
      rotB = acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0, 
		       0, 0, 1);
    }
    else
    {
      rotB = -acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0, 
		       0, 0, 1);
    }
    // b3Printf("calc rotB : rotB = -%f\n", (rotB*180)/M_PI);

    // 骨盤の初期姿勢からの回転分を矯正する（上脚と下脚に回転行列をかける）
    btScalar a_x = pos_data[5][0] - pos_data[4][0];
    btScalar a_y = pos_data[5][1] - pos_data[4][1];
    btScalar a_z = pos_data[5][2] - pos_data[4][2];
    btVector3 a = btVector3(a_x, a_y, a_z);		// 平面内の1ベクトル
    btScalar b_x = pos_data[6][0] - pos_data[5][0];
    btScalar b_y = pos_data[6][1] - pos_data[5][1];
    btScalar b_z = pos_data[6][2] - pos_data[5][2];
    btVector3 b = btVector3(b_x, b_y, b_z);		// 平面内の1ベクトル

    // 上脚，下脚，骨盤を回転させる
    rot_mat = Rz*Ry; 
    btVector3 rot_pelvis = btVector3(rot_mat.tdotx(pelvis), rot_mat.tdoty(pelvis), rot_mat.tdotz(pelvis)); 
    btVector3 rot_a = btVector3(rot_mat.tdotx(a), rot_mat.tdoty(a), rot_mat.tdotz(a)); 
    btVector3 rot_b = btVector3(rot_mat.tdotx(b), rot_mat.tdoty(b), rot_mat.tdotz(b));

    // 骨盤ベクトルを初期骨盤へ回転
    rot_test2 = btVector3(rot_mat.tdotx(pelvis), rot_mat.tdoty(pelvis), rot_mat.tdotz(pelvis));
    norm = rot_test2.norm();
    // b3Printf("baseline:pelvis -> Init-Pelvis :: x = %f, y = %f, z = %f\n", rot_test2.getX()/norm, rot_test2.getY()/norm, rot_test2.getZ()/norm);


    if( judgeCalc(pos_data, 4, 5, 6) ) // 脚のひねりも計算できる時(足首まで推定されている時)
    {
      
      const btVector3 init_plane_crossVec = btVector3(1.f, 0.f, 0.f);

      //
      // calc left_hip_z angle
      // ねじれを計算するため、矯正後の上脚ベクトルをXY平面に投影
      btVector3 project_rot_a = btVector3(rot_a.getX(), rot_a.getY(), 0.0);	
      btVector3 plane_rot_a = -rot_a;
      plane_crossVec = rot_b.cross(plane_rot_a);
      /*
      if(plane_crossVec.getX() <= 0.f )
      {
	b3Printf("change cross order\n");
	plane_crossVec = plane_rot_a.cross(rot_b);
      }
      */
      // b3Printf("plane_crossVec :: x = %f, y = %f, z = %f\n", plane_crossVec.getX(),plane_crossVec.getY(), plane_crossVec.getZ());

      btVector3 project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);
      // 初期脚平面の法線ベクトルとxy平面に投影した法線ベクトルとのなす角を計算
      d = init_plane_crossVec.dot(project_plane_crossVec);
      N = init_plane_crossVec.norm() * project_plane_crossVec.norm();
      temp_crossVec = project_plane_crossVec.cross(init_plane_crossVec);

      if(temp_crossVec.getZ() >= 0.0)
      {
	// 上から見て時計回りに脚をねじっている
	angle_array[9] = -acos(d/N);
      }
      else
      {
	angle_array[9] = acos(d/N);
      }

      Rz = btMatrix3x3(btCos(angle_array[9]), -btSin(angle_array[9]), 0,
		       btSin(angle_array[9]), btCos(angle_array[9]), 0,
		       0, 0, 1);

      b3Printf("left_hip_z = %f\n", (angle_array[9]*180)/M_PI);

      // 
      // calc left_hip_x angle
      // XY平面に投影した脚平面の法線ベクトルと法線ベクトルのなす角を計算
      project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);
      d = project_plane_crossVec.dot(plane_crossVec);
      N = project_plane_crossVec.norm() * plane_crossVec.norm();

      // 初期法線ベクトルと法線ベクトルをXZ平面に投影したベクトルの外積を計算
      project_plane_crossVec = btVector3(plane_crossVec.getX(), 0.f, plane_crossVec.getZ());
      temp_crossVec = project_plane_crossVec.cross(init_plane_crossVec);
      if(temp_crossVec.getY() >= 0)
      {
	// 股を開いている 
	angle_array[10] = -acos(d/N);
      }
      else
      {
	angle_array[10] = acos(d/N);
      }

      Ry = btMatrix3x3(btCos(angle_array[10]), 0, btSin(angle_array[10]),
		       0, 1, 0,
		      -btSin(angle_array[10]), 0, btCos(angle_array[10]));

      b3Printf("left_hip_x = %f\n", (angle_array[10]*180)/M_PI);

      rot_mat = Rz*Ry;

      // 脚平面の法線ベクトルがX軸負方向に一致するか確認
      rot_test1 = btVector3(rot_mat.tdotx(plane_crossVec), rot_mat.tdoty(plane_crossVec), rot_mat.tdotz(plane_crossVec));
      norm = rot_test1.norm();
      // b3Printf("baseline:leg-plane-crossVec -> Positive-X-Axis  x = %f, y = %f, z = %f\n", rot_test1.getX()/norm,rot_test1.getY()/norm, rot_test1.getZ()/norm);

      //
      // calc left_hip_y angle
      // 初期上脚と脚平面の法線ベクトルがX軸負方向に一致している時（XZ平面にある時）のなす角を計算
      btVector3 rot_a2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
      // b3Printf("rot_a2 :: x = %f, y = %f, z = %f\n", rot_a2.getX(), rot_a2.getY(), rot_a2.getZ());
      
      d = init_upper_leg.dot(rot_a2);
      N = init_upper_leg.norm() * rot_a2.norm();

      temp_crossVec = rot_a2.cross(init_upper_leg);

      if(temp_crossVec.getX() >= 0)
      {
	// 振り上げている
	angle_array[11] = -acos(d/N);
      }
      else
      {
	angle_array[11] = acos(d/N);
      }

      Rx = btMatrix3x3(1, 0, 0,
		       0, btCos(angle_array[11]), -btSin(angle_array[11]),
		       0, btSin(angle_array[11]), btCos(angle_array[11]));

      b3Printf("left_hip_y = %f\n", (angle_array[11]*180)/M_PI);

      rot_mat = Rz*Ry*Rx; // Bullet :: Rz*Rx*Ry(回転行列), Z->X->Y(オイラー)

      rot_test2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
      norm = rot_test2.norm();
      // b3Printf("baseline:upper_leg -> Init-UpperLeg :: x = %f, y = %f, z = %f\n", rot_test2.getX()/norm,rot_test2.getY()/norm, rot_test2.getZ()/norm);
    }

#if 1
    // 下脚が推定できておらず、脚平面の法線ベクトルがわからない時
    // 初期上脚と矯正後の上脚の位置から股関節角度を決定する
    else
    {
      const btVector3 negative_Y_axis = btVector3(0.f, -1.f, 0.f);
      //
      // calc left_hip_z angle
      // XY平面に投影した上脚とY軸不方向とのなす角を計算 
      btVector3 project_rot_a = btVector3(rot_a.getX(), rot_a.getY(), 0.0);	
      d = negative_Y_axis.dot(project_rot_a);
      N = negative_Y_axis.norm() * project_rot_a.norm();
      temp_crossVec = negative_Y_axis.cross(project_rot_a);

      if(temp_crossVec.getZ() >= 0.0)
      {
	// 上から見て時計回りに脚をねじっている
	angle_array[9] = acos(d/N); 
      }
      else
      {
        angle_array[9] = -acos(d/N); 
      }

      Rz = btMatrix3x3(btCos(angle_array[9]), -btSin(angle_array[9]), 0,
		       btSin(angle_array[9]), btCos(angle_array[9]), 0, 
		       0, 0, 1);
      
      b3Printf("left_hip_z = %f\n", (angle_array[9]*180)/M_PI);

      //
      // calc left_hip_y angle
      // 上脚ベクトルをYZ平面に投影したベクトルと初期上脚のなす角を計算
      project_rot_a = btVector3(0.f, rot_a.getY(), rot_a.getZ());

      d = init_upper_leg.dot(rot_a);
      N = init_upper_leg.norm() * rot_a.norm();
      temp_crossVec = rot_a.cross(init_upper_leg);

      if(temp_crossVec.getX() >= 0)
      {
	// 振り上げている
	angle_array[11] = -acos(d/N);
      }
      else
      {
	angle_array[11] = acos(d/N);
      }

      Rx = btMatrix3x3(1, 0, 0,
		       0, btCos(angle_array[11]), -btSin(angle_array[11]),
		       0, btSin(angle_array[11]), btCos(angle_array[11]));

      b3Printf("left_hip_y = %f\n", (angle_array[11]*180)/M_PI);

      rot_mat = Rz*Rx;

      // 矯正後の上脚が初期上脚に一致するか確認
      rot_test1 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
      norm = rot_test1.norm();
      // b3Printf("baseline:rot_upper_leg -> Init-UpperLeg :: x = %f, y = %f, z = %f\n", rot_test1.getX()/norm,rot_test1.getY()/norm, rot_test1.getZ()/norm);
    }
#endif
  }
  else
  {
    b3Printf("  Left Hip Is Not Estimated Correctly!\n");
    angle_array[9] = NOT_ESTIMATED;
    angle_array[10] = NOT_ESTIMATED;
    angle_array[11] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc left_knee joint angle
  //
  if( judgeCalc(pos_data, 4, 5, 6) )
  {
    btScalar a_x = pos_data[4][0] - pos_data[5][0];
    btScalar a_y = pos_data[4][1] - pos_data[5][1];
    btScalar a_z = pos_data[4][2] - pos_data[5][2];
    btVector3 a = -btVector3(a_x, a_y, a_z);
    btScalar b_x = pos_data[6][0] - pos_data[5][0];
    btScalar b_y = pos_data[6][1] - pos_data[5][1];
    btScalar b_z = pos_data[6][2] - pos_data[5][2];
    btVector3 b = btVector3(b_x, b_y, b_z);
    d = a.dot(b);
    N = a.norm() * b.norm();
    angle_array[12] = -acos(d/N);
    b3Printf("left_knee = %f\n", (angle_array[12]*180)/M_PI);
  }
  else
  {
    b3Printf("  Left Knee Is Not Estimated Correctly!\n");
    angle_array[12] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1 
  // 
  // calc right_shoulder joint angle
  //
  if( judgeCalc(pos_data, 8, 14, 15) || judgeCalc(pos_data, 11, 14, 15) )
  {
    b3Printf("    --- calc right_shoulder joint angle --- \n");

    const btVector3 init_shoulder_vec = btVector3(1.f, 0.f, 0.f); // 初期肩ベクトル
    const btVector3 init_shoulder_crossVec = btVector3(0.f, 0.f, 1.f); // 初期肩平面の法線ベクトル(今は初期肩がx軸上に存在しているので、z軸を法線ベクトルとする)

    btScalar shoulder_x = 0.f, shoulder_y = 0.f, shoulder_z = 0.f;
    if( judgeCalc(pos_data, 11, 14, 15) )
    {
      // 逆側の肩がちゃんと推定されている時
      shoulder_x = pos_data[11][0] - pos_data[14][0];
      shoulder_y = pos_data[11][1] - pos_data[14][1];
      shoulder_z = pos_data[11][2] - pos_data[14][2];
    }
    else
    {
      // そうでない時、Thoraxを使う
      shoulder_x = pos_data[8][0] - pos_data[14][0];
      shoulder_y = pos_data[8][1] - pos_data[14][1];
      shoulder_z = pos_data[8][2] - pos_data[14][2];
    }

    btVector3 shoulder = btVector3(shoulder_x, shoulder_y, shoulder_z); // 肩ベクトル

    // 初期骨盤平面の法線ベクトルと現在の肩ベクトルのなす角を計算
    //  calc rotA  //
    d = shoulder.dot(init_shoulder_crossVec);
    N = shoulder.norm() * init_shoulder_crossVec.norm();

    btVector3 project_shoulder = btVector3(shoulder.getX(), shoulder.getY(), 0.f);
    // d = shoulder.dot(project_shoulder);
    // N = shoulder.norm() * project_shoulder.norm();

    // project_shoulder = btVector3(shoulder.getX(), 0.f, shoulder.getZ());
    // temp_crossVec = init_shoulder_vec.cross(project_shoulder);

    if(acos(d/N) <= M_PI/2)
    // if(temp_crossVec.getY() <= 0.f)
    {
      // 現在の肩が平面より上側にある時
      rotA = M_PI/2 - acos(d/N);
      Ry = btMatrix3x3(btCos(rotA), 0, btSin(rotA),
		       0, 1, 0,
		      -btSin(rotA), 0, btCos(rotA));
      // b3Printf("calc rotA (above plane) : rotA = %f\n", (rotA*180)/M_PI);
    }
    else
    {
      // 現在の肩が平面より下側にある時
      rotA = acos(d/N) - (M_PI/2);
      Ry = btMatrix3x3(btCos(-rotA), 0, btSin(-rotA),
		       0, 1, 0,
		      -btSin(-rotA), 0, btCos(-rotA));
      // b3Printf("calc rotA (under plane) : rotA = %f\n", (rotA*180)/M_PI);
    }

    // 投影後の肩ベクトルと初期肩ベクトルのなす角を計算
    //  calc rotB  //
    project_shoulder = btVector3(shoulder.getX(), shoulder.getY(), 0.f);
    d = project_shoulder.dot(init_shoulder_vec);
    N = project_shoulder.norm() * init_shoulder_vec.norm();
    // rotBの回転方向を求める
    temp_crossVec = init_shoulder_vec.cross(project_shoulder);

    if(temp_crossVec.getZ() > 0.0)
    {
      rotB = acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
      btSin(rotB), btCos(rotB), 0,
      0, 0, 1);
    }
    else
    {
      rotB = -acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0,
		       0, 0, 1);
    }

    // b3Printf("calc rotB  : rotB = %f\n", (rotB*180)/M_PI);
    
    // 初期姿勢からの回転分を矯正する
    btScalar a_x = pos_data[15][0] - pos_data[14][0];
    btScalar a_y = pos_data[15][1] - pos_data[14][1];
    btScalar a_z = pos_data[15][2] - pos_data[14][2];
    // 上腕ベクトル
    btVector3 a = btVector3(a_x, a_y, a_z);		// 平面内の1ベクトル
    btScalar b_x = pos_data[16][0] - pos_data[15][0];
    btScalar b_y = pos_data[16][1] - pos_data[15][1];
    btScalar b_z = pos_data[16][2] - pos_data[15][2];
    // 前腕ベクトル
    btVector3 b = btVector3(b_x, b_y, b_z);		// 平面内の1ベクトル

    rot_mat = Rz*Ry;

    btVector3 rot_shoulder = btVector3(rot_mat.tdotx(shoulder), rot_mat.tdoty(shoulder), rot_mat.tdotz(shoulder));
    btVector3 rot_a = btVector3(rot_mat.tdotx(a), rot_mat.tdoty(a), rot_mat.tdotz(a));
    btVector3 rot_b = btVector3(rot_mat.tdotx(b), rot_mat.tdoty(b), rot_mat.tdotz(b));

    // 肩が初期肩に一致するか確認
    rot_test2 = btVector3(rot_mat.tdotx(shoulder), rot_mat.tdoty(shoulder), rot_mat.tdotz(shoulder));
    norm = rot_test2.norm();
    // b3Printf("baseline:shoulder -> Init-Shoulder :: x = %f y = %f z = %f\n", rot_test2.getX()/norm,rot_test2.getY()/norm, rot_test2.getZ()/norm);


    // 関節角度計算
    const btVector3 init_upper_arm = btVector3(0.f, 0.f, -1.f);
    const btVector3 negative_Y_axis = btVector3(0.f, -1.f, 0.f);

    // 
    // calc right_shoulder_z angle
    btVector3 project_rot_a = btVector3(rot_a.getX(), rot_a.getY(), 0.0);
    btVector3 plane_rot_a = -rot_a;
    plane_crossVec = rot_b.cross(plane_rot_a);
    btVector3 project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);

    // Y軸負方向（初期法線ベクトル）とxy平面に投影した法線ベクトルとのなす角を計算
    d = negative_Y_axis.dot(project_plane_crossVec);
    N = negative_Y_axis.norm() * project_plane_crossVec.norm();
    temp_crossVec = project_plane_crossVec.cross(negative_Y_axis);
    // temp_crossVec = negative_Y_axis.cross(project_plane_crossVec);

    if(temp_crossVec.getZ() >= 0.0)
    {
      // 上から見て反時計回りに腕をねじっている
      angle_array[15] = -acos(d/N);
      // angle_array[15] = acos(d/N);
    }
    else
    {
      angle_array[15] = acos(d/N);
      // angle_array[15] = -acos(d/N);
    }

    Rz = btMatrix3x3(btCos(angle_array[15]), -btSin(angle_array[15]), 0,
		     btSin(angle_array[15]), btCos(angle_array[15]), 0,
		     0, 0, 1);

    b3Printf("right_shoulder_z = %f\n", (angle_array[15]*180)/M_PI);

    //
    // calc right_shoulder_y angle
    // xy平面に投影した腕平面の法線ベクトルと法線ベクトルのなす角を計算
    d = project_plane_crossVec.dot(plane_crossVec);
    N = project_plane_crossVec.norm() * plane_crossVec.norm();

    // 初期上腕とYZ平面に投影した矯正後の上腕の外積を計算
    project_rot_a = btVector3(0.f, rot_a.getY(), rot_a.getZ());
    temp_crossVec = project_rot_a.cross(init_upper_arm);

    if(temp_crossVec.getX() >= 0)
    {
      // 振り上げている
      angle_array[16] = -acos(d/N);
      // btScalar margin = (20.f*M_PI)/180.f;
      // angle_array[16] = -(acos(d/N)+margin);
    }
    else
    {
      angle_array[16] = acos(d/N);
    }

    Rx = btMatrix3x3(1, 0, 0,
 		     0, btCos(angle_array[16]), -btSin(angle_array[16]),
		     0, btSin(angle_array[16]), btCos(angle_array[16]));

    b3Printf("right_shoulder_y = %f\n", (angle_array[16]*180)/M_PI);

    rot_mat = Rz*Rx;

    // 腕平面の法線ベクトルがY軸負方向に一致するか確認
    rot_test1 = btVector3(rot_mat.tdotx(plane_crossVec), rot_mat.tdoty(plane_crossVec), rot_mat.tdotz(plane_crossVec));
    norm = rot_test1.norm();
    // b3Printf("baseline:arm-plane-crossVec -> Negative-Y-Axis  x = %f, y = %f, z = %f\n", rot_test1.getX()/norm, rot_test1.getY()/norm, rot_test1.getZ()/norm);

    //
    // calc right_shoulder_x angle
    // 初期上腕と腕平面の法線ベクトルがY軸負方向に一致している時（xz平面にある時）のなす角を計算
    btVector3 rot_a2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
    d = init_upper_arm.dot(rot_a2);
    N = init_upper_arm.norm() * rot_a2.norm();

    temp_crossVec = rot_a2.cross(init_upper_arm);

    if(temp_crossVec.getY() >= 0)
    {
      // 脇を開いている
      angle_array[17] = -acos(d/N);
      angle_array[17] = acos(d/N);
    }
    else
    {
      angle_array[17] = acos(d/N);
      angle_array[17] = -acos(d/N);
    }

    Ry = btMatrix3x3(btCos(-angle_array[17]), 0, btSin(-angle_array[17]),
		     0, 1, 0,
		    -btSin(-angle_array[17]), 0, btCos(-angle_array[17]));

    b3Printf("right_shoulder_x = %f\n", (angle_array[17]*180)/M_PI);

    rot_mat = Rz*Rx*Ry; // Bullet :: Rz*Ry*Rx(回転行列), Z->Y->X(オイラー)

    rot_test2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
    norm = rot_test2.norm();
    // b3Printf("baseline:upper_arm -> Init-UpperArm  x = %f, y = %f, z = %f\n", rot_test2.getX()/norm,rot_test2.getY()/norm, rot_test2.getZ()/norm);
  }
  else
  {
    b3Printf("  Right Shoulder Is Not Estimated Correctly!\n");
    angle_array[15] = NOT_ESTIMATED;
    angle_array[16] = NOT_ESTIMATED;
    angle_array[17] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc right_elbow joint angle
  //
  if( judgeCalc(pos_data, 14, 15, 16) )
  {

    b3Printf("    --- calc right_elbow joint angle --- \n");
    btScalar a_x = pos_data[14][0] - pos_data[15][0];
    btScalar a_y = pos_data[14][1] - pos_data[15][1];
    btScalar a_z = pos_data[14][2] - pos_data[15][2];
    btVector3 a = -btVector3(a_x, a_y, a_z);
    btScalar b_x = pos_data[16][0] - pos_data[15][0];
    btScalar b_y = pos_data[16][1] - pos_data[15][1];
    btScalar b_z = pos_data[16][2] - pos_data[15][2];
    btVector3 b = btVector3(b_x, b_y, b_z);
    d = a.dot(b);
    N = a.norm() * b.norm();
    angle_array[18] = acos(d/N);
    b3Printf("right_elbow = %f\n", (angle_array[18]*180)/M_PI);
  }
  else
  {
    b3Printf("  Righe Elbow Is Not Estimated Correctly!\n");
    angle_array[18] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc left_shoulder joint angle
  //
  if( judgeCalc(pos_data, 8, 11, 12) || judgeCalc(pos_data, 14, 11, 12) ) 
  {
    b3Printf("    --- calc left_shoulder joint angle --- \n");

    const btVector3 init_shoulder_vec = btVector3(-1.f, 0.f, 0.f); // 初期肩ベクトル
    const btVector3 init_shoulder_crossVec = btVector3(0.f, 0.f, 1.f); // 初期肩平面の法線ベクトル(今は初期肩がx軸上に存在しているので、z軸を法線ベクトルとする）

    btScalar shoulder_x = 0.f, shoulder_y = 0.f, shoulder_z = 0.f;
    if( judgeCalc(pos_data, 14, 11, 12) ) 
    {
      shoulder_x = pos_data[14][0] - pos_data[11][0];
      shoulder_y = pos_data[14][1] - pos_data[11][1];
      shoulder_z = pos_data[14][2] - pos_data[11][2];
    }
    else
    {
      shoulder_x = pos_data[8][0] - pos_data[11][0];
      shoulder_y = pos_data[8][1] - pos_data[11][1];
      shoulder_z = pos_data[8][2] - pos_data[11][2];
    }

    btVector3 shoulder = btVector3(shoulder_x, shoulder_y, shoulder_z);		// 肩ベクトル

    // 初期肩平面の法線ベクトルと現在の肩ベクトルのなす角を計算
    //  calc rotA  //
    d = shoulder.dot(init_shoulder_crossVec);
    N = shoulder.norm() * init_shoulder_crossVec.norm();

    btVector3 project_shoulder = btVector3(shoulder.getX(), 0.f, shoulder.getZ());
    if(acos(d/N) <= M_PI/2)
    // if(temp_crossVec.getY() >= 0.0)
    {
      // 現在の肩が平面より上側にある時
      rotA = acos(d/N) - (M_PI/2);
      Ry = btMatrix3x3(btCos(-rotA), 0, btSin(-rotA),
		       0, 1, 0, 
		      -btSin(-rotA), 0, btCos(-rotA));
      // b3Printf("calc rotA (above plane) : rotA = %f\n", (rotA*180)/M_PI);
    }
    else
    {
      // 現在の肩が平面より下側にある時
      rotA = M_PI/2 - acos(d/N);
      Ry = btMatrix3x3(btCos(rotA), 0, btSin(rotA),
		       0, 1, 0, 
		      -btSin(rotA), 0, btCos(rotA));
      // b3Printf("calc rotA (under plane) : rotA = %f\n", (rotA*180)/M_PI);
    }

    // 投影後の肩ベクトルと初期肩ベクトルのなす角を計算
    //  calc rotB  //
    project_shoulder = btVector3(shoulder.getX(), shoulder.getY(), 0.f);
    d = project_shoulder.dot(init_shoulder_vec);
    N = project_shoulder.norm() * init_shoulder_vec.norm();
    // rotBの回転方向を求める
    temp_crossVec = init_shoulder_vec.cross(project_shoulder);
    
    if(temp_crossVec.getZ() > 0.0)
    {
      rotB = acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0, 
		       0, 0, 1);
    }
    else
    {
      rotB = -acos(d/N);
      Rz = btMatrix3x3(btCos(rotB), -btSin(rotB), 0,
		       btSin(rotB), btCos(rotB), 0, 
		       0, 0, 1);
    }

    // b3Printf("calc rotB : rotB = %f\n", (rotB*180)/M_PI);

    // 初期姿勢からの回転分を矯正する（上腕（と前腕）に回転行列をかける）
    btScalar a_x = pos_data[12][0] - pos_data[11][0];
    btScalar a_y = pos_data[12][1] - pos_data[11][1];
    btScalar a_z = pos_data[12][2] - pos_data[11][2];
    // 上腕ベクトル
    btVector3 a = btVector3(a_x, a_y, a_z);		// 平面内の1ベクトル
    // 前腕ベクトル
    btScalar b_x = pos_data[13][0] - pos_data[12][0];
    btScalar b_y = pos_data[13][1] - pos_data[12][1];
    btScalar b_z = pos_data[13][2] - pos_data[12][2];
    btVector3 b = btVector3(b_x, b_y, b_z);		

    rot_mat = Rz*Ry;

    btVector3 rot_shoulder = btVector3(rot_mat.tdotx(shoulder), rot_mat.tdoty(shoulder), rot_mat.tdotz(shoulder));
    btVector3 rot_a = btVector3(rot_mat.tdotx(a), rot_mat.tdoty(a), rot_mat.tdotz(a));
    btVector3 rot_b = btVector3(rot_mat.tdotx(b), rot_mat.tdoty(b), rot_mat.tdotz(b));

    rot_test2 = btVector3(rot_mat.tdotx(shoulder), rot_mat.tdoty(shoulder), rot_mat.tdotz(shoulder));
    norm = rot_test2.norm();
    // b3Printf("baseline:shoulder -> Init-Shoulder :: x = %f y = %f z = %f\n", rot_test2.getX()/norm, rot_test2.getY()/norm, rot_test2.getZ()/norm);


    // 関節角度計算
    const btVector3 init_upper_arm = btVector3(0.f, 0.f, -1.f); 
    const btVector3 negative_Y_axis = btVector3(0.f, -1.f, 0.f);

    // 
    // calc left_shoulder_z angle
    btVector3 project_rot_a = btVector3(rot_a.getX(), rot_a.getY(), 0.0);	
    btVector3 plane_rot_a = -rot_a;
    plane_crossVec = plane_rot_a.cross(rot_b);
    btVector3 project_plane_crossVec = btVector3(plane_crossVec.getX(), plane_crossVec.getY(), 0.0);
    // Y軸負方向（初期法線ベクトル）とxy平面に投影した法線ベクトルとのなす角を計算
    d = negative_Y_axis.dot(project_plane_crossVec);
    N = negative_Y_axis.norm() * project_plane_crossVec.norm();
    temp_crossVec = negative_Y_axis.cross(project_plane_crossVec);

    // if(a_x > 0)
    if(temp_crossVec.getZ() >= 0.0)
    {
      // 上から見て時計回りに腕をねじっている
      angle_array[19] = acos(d/N); 
    }
    else
    {
      angle_array[19] = -acos(d/N); 
    }


    Rz = btMatrix3x3(btCos(angle_array[19]), -btSin(angle_array[19]), 0,
		     btSin(angle_array[19]), btCos(angle_array[19]), 0, 
		     0, 0, 1);

    b3Printf("left_shoulder_z = %f\n", (angle_array[19]*180)/M_PI);

    //
    // calc left_shoulder_y angle
    // xy平面に投影した腕平面の法線ベクトルと法線ベクトルのなす角を計算
    d = project_plane_crossVec.dot(plane_crossVec);
    N = project_plane_crossVec.norm() * plane_crossVec.norm();

    // 初期上腕とYZ平面に投影した矯正後の上腕の外積を計算
    project_rot_a = btVector3(0.f, rot_a.getY(), rot_a.getZ());
    temp_crossVec = project_rot_a.cross(init_upper_arm);

    if(temp_crossVec.getX() >= 0)
    {
      // 振り上げている
      angle_array[20] = -acos(d/N);
    }
    else
    {
      angle_array[20] = acos(d/N);
    }

    Rx = btMatrix3x3(1, 0, 0,
		     0, btCos(angle_array[20]), -btSin(angle_array[20]),
		     0, btSin(angle_array[20]), btCos(angle_array[20]));

    b3Printf("left_shoulder_y = %f\n", (angle_array[20]*180)/M_PI);

    rot_mat = Rz*Rx;

    // 腕平面の法線ベクトルがY軸負方向に一致するか確認
    rot_test1 = btVector3(rot_mat.tdotx(plane_crossVec), rot_mat.tdoty(plane_crossVec), rot_mat.tdotz(plane_crossVec));
    norm = rot_test1.norm();
    // b3Printf("baseline:arm-plane-crossVec -> Negative-Y-Axis  x = %f, y = %f, z = %f\n", rot_test1.getX()/norm, rot_test1.getY()/norm, rot_test1.getZ()/norm);


    //
    // calc left_shoulder_x angle
    // 初期上腕と腕平面の法線ベクトルがY軸負方向に一致している時（xz平面にある時）のなす角を計算
    btVector3 rot_a2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));

    d = init_upper_arm.dot(rot_a2);
    N = init_upper_arm.norm() * rot_a2.norm();

    temp_crossVec = rot_a2.cross(init_upper_arm);

    if(temp_crossVec.getY() >= 0)
    {
      // 脇を開いている 
      angle_array[21] = -acos(d/N);
    }
    else
    {
      angle_array[21] = acos(d/N);
    }

    Ry = btMatrix3x3(btCos(angle_array[21]), 0, btSin(angle_array[21]),
		     0, 1, 0,
		    -btSin(angle_array[21]), 0, btCos(angle_array[21]));

    b3Printf("left_shoulder_x = %f\n", (angle_array[21]*180)/M_PI);

    rot_mat = Rz*Rx*Ry; // Bullet :: Rz*Ry*Rx(回転行列), Z->Y->X(オイラー)

    // 
    rot_test2 = btVector3(rot_mat.tdotx(rot_a), rot_mat.tdoty(rot_a), rot_mat.tdotz(rot_a));
    norm = rot_test2.norm();
    // b3Printf("baseline:upper_arm -> Init-UpperArm :: x = %f y = %f z = %f\n", rot_test2.getX()/norm, rot_test2.getY()/norm, rot_test2.getZ()/norm);
  }
  else
  {
    b3Printf("  Left Shoulder Is Not Estimated Correctly!\n");
    angle_array[19] = NOT_ESTIMATED;
    angle_array[20] = NOT_ESTIMATED;
    angle_array[21] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

#if 1
  //
  // calc left_elbow joint angle
  //
  if( judgeCalc(pos_data, 11, 12, 13) )
  {
    btScalar a_x = pos_data[11][0] - pos_data[12][0];
    btScalar a_y = pos_data[11][1] - pos_data[12][1];
    btScalar a_z = pos_data[11][2] - pos_data[12][2];
    btVector3 a = -btVector3(a_x, a_y, a_z);
    btScalar b_x = pos_data[13][0] - pos_data[12][0];
    btScalar b_y = pos_data[13][1] - pos_data[12][1];
    btScalar b_z = pos_data[13][2] - pos_data[12][2];
    btVector3 b = btVector3(b_x, b_y, b_z);
    d = a.dot(b);
    N = a.norm() * b.norm();
    angle_array[22] = acos(d/N);
    b3Printf("left_elbow = %f\n", (angle_array[22]*180)/M_PI);
  }
  else
  {
    b3Printf("  Left Elbow Is Not Estimated Correctly!\n");
    angle_array[22] = NOT_ESTIMATED;
  }
#endif

  // ------------------------------------------------------------

  return;
}


std::map<int, std::string> joint_map = {
  {0, "abdomen_z"},
  {1, "abdomen_y"},
  {2, "abdomen_x"},
  {3, "right_hip_z"},
  {4, "right_hip_x"},
  {5, "right_hip_y"},
  {6, "right_knee"},
  {7, "right_ankle_y"},
  {8, "right_ankle_x"},
  {9, "left_hip_z"},
  {10, "left_hip_x"},
  {11, "left_hip_y"},
  {12, "left_knee"},
  {13, "left_ankle_y"},
  {14, "left_ankle_x"},
  {15, "right_shoulder_z"},
  {16, "right_shoulder_y"},
  {17, "right_shoulder_x"},
  {18, "right_elbow"},
  {19, "left_shoulder_z"},
  {20, "left_shoulder_y"},
  {21, "left_shoulder_x"},
  {22, "left_elbow"}
};



// step Simulation
void Nursing::stepSimulation(float deltaTime)
{
  if (m_dynamicsWorld)
  {
    // btVector3 gravity(0, 0, -10);
    // gravity[m_upAxis] = m_grav;
    // m_dynamicsWorld->setGravity(gravity);

    btScalar fixedTimeStep = 1. / 240.f;
    // btScalar fixedTimeStep = 1. / 120.f;
    
    static int count = 0;
    static int keypoint_frame = 0; // 現在のフレーム数
    const static int data_size = vstr.size();
    static bool once = true; // 一度だけSimulationの終了を端末上に通知するための変数
    static bool input_angle = false;

    const static int start_frame = 0;
    // const static int stop_frame = vstr.size()/2; // 32
    const static int stop_frame = 0; // 32
    // const static int stop_frame = 10; 

    for(int n = 0; n < m_datas.size(); n++)
    {
      struct ImportMJCFInternalData* humanoid_data = m_datas[n];


      bool flag = (data_size > keypoint_frame*num_humanoid + n) ? true : false; 
      // if( keypoint_frame*num_humanoid+n == 0)
      if( count == 0 && once)
      {
	b3Printf("  ~~~~~ Start (Nursing) Simulation ~~~~~ \n");
	// std::string str;
	// b3Printf(" Input Angle? -> \n");
	// getline(std::cin, str);

      }

      if(!flag && once)
      {
	b3Printf("  ~~~~~ Finish (Nursing) Simulation ~~~~~ \n");

	if( n == 0 )
	  for(int i = 0; i < 23; i++)
	    humanA_angle_array[i] = 0.f;
	else if( n == 1 )
	  for(int i = 0; i < 23; i++)
	    humanB_angle_array[i] = 0.f;

	once = false;
      }

      if(((count & 0x0) == 0 && flag && count != 0))
      // if(flag)
      {
	// b3Printf("count = %d\n", count);


	if(once && n == 0)
  	  b3Printf("========== frame : %d ========== \n", keypoint_frame);

	if(start_frame <= keypoint_frame && keypoint_frame <= stop_frame)
	{
	  if( n == 0 )
	  {
            // calcJointAngle(humanA_angle_array, keypoint_frame, n); 

	    /*
	    // 初期値を設定
	    if( keypoint_frame == 0 )
	      for (int i = 0; i < humanoid_data->m_numMotors; i++)
	        humanA_before_angle_array[i] = humanA_angle_array[i];
	    */
              
	  }
	  else
	  {
            calcJointAngle(humanB_angle_array, keypoint_frame, n); 

	    /*
	    // 初期値を設定
	    if( keypoint_frame == 0 )
	      for (int i = 0; i < humanoid_data->m_numMotors; i++)
	        humanB_before_angle_array[i] = humanB_angle_array[i];
            */
	  }
	  // once = false;
	}

	for (int i = 0; i < humanoid_data->m_numMotors; i++)
	{
	  if (humanoid_data->m_jointMotors[i])
	  {
	    btScalar pos;
	    // b3Printf("human = %d , i = %d\n", n, i);
#ifdef SIMULATION
	    // Joint角度が計算できない時とあまりに前回の結果と違う時、前回の結果を使う
	    /*
	    if( n == 0 )
	    {
              if(angle_array[i] == NOT_ESTIMATED || btFabs(angle_array[i] - humanA_before_angle_array[i] > M_PI/18))
	        angle_array[i] = humanA_before_angle_array[i];
	    }
	    else
	    {
	      if(angle_array[i] == NOT_ESTIMATED || btFabs(angle_array[i] - humanB_before_angle_array[i] > M_PI/18))
	        angle_array[i] = humanB_before_angle_array[i];
	    }
	    */
	   

	    if( n == 0 )
	    {

	      // 関節角度が計算できている時
              if(humanA_angle_array[i] != NOT_ESTIMATED)
	      {
		/*
	        if(btFabs(btFabs(humanA_angle_array[i]) - btFabs(humanA_before_angle_array[i])) > M_PI/6)
	          pos = humanA_before_angle_array[i];
		else
		*/
		{
	          pos = humanA_angle_array[i];
	          // 関節角度を保存
	          humanA_before_angle_array[i] = humanA_angle_array[i];
		}
	      }
	      else
	      {
	        pos = humanA_before_angle_array[i];
		b3Printf(" ====> use before angle\n");
	      }

	    }
	    else
	    {

              if(humanB_angle_array[i] != NOT_ESTIMATED)
	      {
		/*
	        if(btFabs(btFabs(humanB_angle_array[i]) - btFabs(humanB_before_angle_array[i])) > M_PI/6)
	          pos = humanB_before_angle_array[i];
		else
		*/
		{
	          pos = humanB_angle_array[i];
	          // 関節角度を保存
	          humanB_before_angle_array[i] = humanB_angle_array[i];
		}
	      }
	      else
	      {
	        pos = humanB_before_angle_array[i];
		b3Printf(" ====> use before angle\n");
	      }

	    }


	    /*
	    // 変化量があまりに大きい時、推定が逆になっている
	    if(btFabs(angle_array[6] - before_angle_array[6]) > M_PI/6)
	    {
	      if( n == 0 )
		angle_array[i] = humanB_angle_array[i];
              if( n == 1 )
		angle_array[i] = humanA_angle_array[i];
	    }
	    */
	    

	    // btScalar pos = angle_array[i];


	    /*
 	    if(i == 15)
	      pos = angle_array[17];
	    else if(i == 17)
	      pos = angle_array[15];
	      */
               
	    if(pos && flag && once)
	    {
	      // b3Printf("human %d :: %s = %f\n", n, joint_map[i].c_str(), (angle_array[i]*180)/M_PI);
	    }
	    else if(!pos && input_angle)
	    {
	      std::string str_angle;
	      b3Printf("  Please Input human%d %s Angle -> ", n, joint_map[i].c_str());
	      getline(std::cin, str_angle);
	      btScalar angle = (std::stof(str_angle)*M_PI)/180.f;
	      pos = angle;
	    }
#else
	    btScalar pos = humanoid_data->m_motorTargetPositions[i];
#endif

	      
	    int link = humanoid_data->m_jointMotors[i]->getLinkA();
	    btScalar lowerLimit = humanoid_data->m_mb->getLink(link).m_jointLowerLimit;
	    btScalar upperLimit = humanoid_data->m_mb->getLink(link).m_jointUpperLimit;
	    if (lowerLimit < upperLimit)
	    {
	      btClamp(pos, lowerLimit, upperLimit);
	    }

	    humanoid_data->m_jointMotors[i]->setPositionTarget(pos);
	  }
	}

	if(n == (m_datas.size() - 1))
	  keypoint_frame++;
	// if(keypoint_frame == stop_frame)
          // break;
      }
    }


    if(DrawContactForceFlag)
    {
      DrawContactForce(fixedTimeStep);
    }
    if(DrawMotorForceFlag)
    {
      if(count > 1)
        DrawMotorForce(fixedTimeStep);
    }
    if(DrawSoftForceFlag)
    {
      DrawSoftBodyAppliedForce(fixedTimeStep);
    }

    count++;

    //the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
    m_dynamicsWorld->stepSimulation(deltaTime, 10, fixedTimeStep);
    // m_dynamicsWorld->stepSimulation(deltaTime);
   
    // b3Printf("deltaTime = %f\n", deltaTime);
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
    // int flag = btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT;
    int flag = btCollisionObject::CF_STATIC_OBJECT;
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
	  // b3Printf("contact force = %f, %f(imp*dt)\n", contact_force, contact_force); 
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
	  else if( MAX_CONTACT_FORCE/3 <= contact_force && contact_force < 2*MAX_CONTACT_FORCE/3)
             color = yellow;
	  else if( 2*MAX_CONTACT_FORCE/3 <= contact_force && contact_force < MAX_CONTACT_FORCE)
            color = red;
	  else if( MAX_CONTACT_FORCE <= contact_force)
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

void Nursing::DrawMotorForce(btScalar fixedTimeStep)
{ // 黒(0,0,0)
  for(int n = 0; n < m_datas.size(); n++)
  {
    struct ImportMJCFInternalData* humanoid_data = m_datas[n];

    for(int i = 0; i < humanoid_data->m_numMotors; i++)
    {
      // b3Printf("m_numMotors = %d\n", humanoid_data->m_numMotors);
      float motor_force = 0.0;
      if(humanoid_data->m_jointMotors[i])
      {
	// Show Joint Name
	// std::string* jointName = new std::string(humanoid_data->m_mb->getLink(i).m_jointName);
	// b3Printf("i = %d : JointName is %s\n", i, jointName->c_str());
	// 下のif文を付けないとsignal6でプログラムが終了する
	// if(humanoid_data->m_jointMotors[i]->getDataSize())
	{
	  motor_force = humanoid_data->m_jointMotors[i]->getAppliedImpulse(0) / fixedTimeStep; 

	  btTransform tr;

	  {
	    if(MotorJointNames[i] == std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkA()).m_jointName)){
	      tr = humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkA()).m_cachedWorldTransform;
	      std::string* LinkAName = new std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkA()).m_linkName);
	      std::string* JointAName = new std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkA()).m_jointName);

	      // b3Printf("LinkA : %s  %f, %f, %f\n", LinkAName->c_str(), tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
	      // b3Printf("jointName = %s\n", JointAName->c_str());
	    }
	    else if(humanoid_data->m_jointMotors[i]->getLinkB() && MotorJointNames[i] == std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkB()).m_jointName)){
	      tr = humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkB()).m_cachedWorldTransform;
	      std::string* LinkBName = new std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkB()).m_linkName);
	      std::string* JointBName = new std::string(humanoid_data->m_mb->getLink(humanoid_data->m_jointMotors[i]->getLinkB()).m_jointName);

	      // b3Printf("LinkB : %s  %f, %f, %f\n", LinkBName->c_str(), tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
	      // b3Printf("jointName = %s\n", JointBName->c_str());
	    }
	  }
	  btVector3 pos = tr.getOrigin();
	  btScalar pointSize = 25;
	  btScalar delta_color = motor_force / MAX_JOINTMOTOR_TORQUE;
	  // btVector3 color = btVector3(1-delta_color, 1-delta_color, 1-delta_color);
	  // b3Printf("Draw human = %d, joint = %d\n", n, i);
	  // btVector3 color = white;
	  btVector3 color;
	  if( 0.f < motor_force && motor_force < MAX_JOINTMOTOR_TORQUE/3)
	    color = green;
	  else if( MAX_JOINTMOTOR_TORQUE/3 <= motor_force && motor_force < 2*MAX_JOINTMOTOR_TORQUE/3)
            color = yellow;
	  else if( 2*MAX_JOINTMOTOR_TORQUE/3 <= motor_force && motor_force < MAX_JOINTMOTOR_TORQUE)
            color = red;
	  else if( MAX_JOINTMOTOR_TORQUE <= motor_force)
	  {
            color = black;
	    // b3Printf("	joint torque = %f\n", motor_force);
	  }

	  // color = black;

	  if(motor_force > 0.f)
	    m_guiHelper->getRenderInterface()->drawPoint(pos, color, pointSize);
	}
      }
    }
  }
}


btScalar max_impulse = 0;
btScalar max_total_impulse = 0;
btScalar max_force = 0;
btScalar max_total_force = 0;
#if 1
void Nursing::DrawSoftBodyAppliedForce(btScalar fixedTimeStep)
{ // 黃(255, 255, 0)
  btSoftMultiBodyDynamicsWorld* softWorld = getSoftDynamicsWorld();

  btScalar total_f = 0;
  btScalar total_ima = 0;
  btScalar pointSize = 10;
  btVector3 pos = btVector3(0, 0, 0);
#if 1
  if(softWorld)
  {
    for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
    {
      btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
      btSoftBody::Node* n = 0;
      btCollisionObject* col_obj = 0;
      btSoftBody::RContact rcontact;
      if(psb)
      {
        int rcontact_node_size = psb->m_rcontacts.size();
	btCollisionObject* col_ob = 0;
	int flag = btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT;
	for(int j = 0; j < rcontact_node_size; j++)
	{
	  rcontact = psb->m_rcontacts[j];
	  col_obj = const_cast<btCollisionObject*>(rcontact.m_cti.m_colObj);
	  btVector3 bary = rcontact.m_cti.m_bary;
	  if(!(col_obj->getCollisionFlags() & flag))
	  {
	    n = rcontact.m_node;
	    pos = n->m_x;
	    // btScalar delta_color = / MAX_SOFTBODY_FORCE;
            btVector3 color = btVector3(1, 1, 0);
	    btVector3 f = n->m_f;
	    total_f += f.length();
	    btScalar im = n->m_im;
	    btScalar c2 = rcontact.m_c2;
	    btVector3 vel = n->m_v;
	    btVector3 delta = rcontact.t2 - rcontact.t1;
	    total_ima += c2;
	    // b3Printf("m_x(node position) = %f, %f, %f\n", pos.x(), pos.y(), pos.z());
	    // b3Printf("m_f(node force accumulator) = %f, %f, %f length = %f\n", f.x(), f.y(), f.z(), f.length()); // 0.0
	    // b3Printf("m_im(node invers mass) = %f\n", im);
	    // b3Printf("m_bary(Barycentric weights for faces) = %f, %f, %f length = %f\n", bary.x(), bary.y(), bary.z(), bary.length()); // 0.0
	    // b3Printf("m_c2(ima*dt) = %f\n", c2); // 全ノードで一定(7.225001) 剛体の重さを変えても変わらなかった
	    // b3Printf("m_c2(ima*dt) = %f\n", c2/dt); 
	    // b3Printf("m_v(node velocity) = %f, %f, %f length = %f\n", vel.x(), vel.y(), vel.z(), vel.length()); // 
	    // b3Printf("delta impuse(t2-t1) = %f, %f, %f length = %f\n", delta.x(), delta.y(), delta.z(), delta.length()); // 
	    // b3Printf("softbody force = %f\n", psb_force);
	    // m_guiHelper->getRenderInterface()->drawPoint(pos, color, pointSize);
	  }
	}
#if 0
	if(total_f)
	  b3Printf("total_f = %f\n", total_f);
	if(total_ima)
	{
	  b3Printf("total_ima = %f\n", total_ima);
	  b3Printf("inv_total_ima = %f\n", 1/total_ima);
	  b3Printf("total_ima(/dt) = %f\n", total_ima/dt);
	  b3Printf("inv_total_ima(/dt) = %f\n", 1/total_ima/dt);
	}
#endif
      }
    }
  }
#endif


  btScalar kst = 1;
  // btScalar ti = isolve / (btScalar)m_cfg.piterations; isolve:0 ~ (piterations-1)
  // btSoftBody::Impulse impulse;

  if(softWorld)
  {
    for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
    {
      btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
      BT_PROFILE("PSolve_RContacts");
      const btScalar dt = psb->m_sst.sdt;
      const btScalar mrg = psb->getCollisionShape()->getMargin();
      btMultiBodyJacobianData jacobianData;
      btScalar total_impulse = 0;
      for (int i = 0, ni = psb->m_rcontacts.size(); i < ni; ++i)
      {
	const btSoftBody::RContact& c = psb->m_rcontacts[i];
	const btSoftBody::sCti& cti = c.m_cti;
	if (cti.m_colObj->hasContactResponse())
	{
	  btVector3 va(0, 0, 0);
	  btRigidBody* rigidCol = 0;
	  btMultiBodyLinkCollider* multibodyLinkCol = 0;
	  btScalar* deltaV;

	  int flag = btCollisionObject::CO_RIGID_BODY | btCollisionObject::CO_RIGID_BODY;
	  if (cti.m_colObj->getInternalType() & flag)
	  {
	    rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
	    va = rigidCol ? rigidCol->getVelocityInLocalPoint(c.m_c1) * dt : btVector3(0, 0, 0);
	  }
	  else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
	  {
	    multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
	    if (multibodyLinkCol)
	    {
	      const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
	      jacobianData.m_jacobians.resize(ndof);
	      jacobianData.m_deltaVelocitiesUnitImpulse.resize(ndof);
	      btScalar* jac = &jacobianData.m_jacobians[0];

	      multibodyLinkCol->m_multiBody->fillContactJacobianMultiDof(multibodyLinkCol->m_link, c.m_node->m_x, cti.m_normal, jac, jacobianData.scratch_r, jacobianData.scratch_v, jacobianData.scratch_m);
	      deltaV = &jacobianData.m_deltaVelocitiesUnitImpulse[0];
	      multibodyLinkCol->m_multiBody->calcAccelerationDeltasMultiDof(&jacobianData.m_jacobians[0], deltaV, jacobianData.scratch_r, jacobianData.scratch_v);

	      btScalar vel = 0.0;
	      for (int j = 0; j < ndof; ++j)
	      {
		vel += multibodyLinkCol->m_multiBody->getVelocityVector()[j] * jac[j];
	      }
	      va = cti.m_normal * vel * dt;
	    }
	  }

	  const btVector3 vb = c.m_node->m_x - c.m_node->m_q;
	  const btVector3 vr = vb - va;
	  const btScalar dn = btDot(vr, cti.m_normal);
	  if (dn <= SIMD_EPSILON)
	  {
	    const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
	    const btVector3 fv = vr - (cti.m_normal * dn);
	    // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
	    const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst);
	    c.m_node->m_x -= impulse * c.m_c2;
	    if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY && 
		!(cti.m_colObj->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT))
	    {
	      if (rigidCol)
	      {
		btTransform trans = cti.m_colObj->getWorldTransform ();
		btVector3 rigid_center = trans.getOrigin();
		// rigidCol->applyImpulse(impulse, c.m_c1);
		btScalar pointSize = 30;
		// btVector3 pos = c.m_c1;
		btVector3 pos = rigid_center + c.m_c1;
		// btVector3 pos = c.m_node->m_x;
		if(impulse.length() > 0.f)
		{
		  // b3Printf("impulse = %f, %f, %f length = %f\n", impulse.x(), impulse.y(), impulse.z(), impulse.length()); 
		  // b3Printf("c.m_c1 = %f, %f, %f\n",c.m_c1.x(), c.m_c1.y(), c.m_c1.z()); // Anchorの位置を返す
	          // b3Printf("m_x2(node position) = %f, %f, %f\n", c.m_node->m_x.x(), c.m_node->m_x.y(), c.m_node->m_x.z());

		  btScalar force = impulse.length() / fixedTimeStep;

		  // btVector3 color = white;
		  btVector3 color = green;
		  if( 0.f < force && force < MAX_SOFTBODY_FORCE/3)
		    color = green;
		  else if( MAX_SOFTBODY_FORCE/3 <= force && force < 2*MAX_SOFTBODY_FORCE/3)
		     color = yellow;
		  else if( 2*MAX_SOFTBODY_FORCE/3<= force && force < MAX_SOFTBODY_FORCE)
		    color = red;
		  else if( MAX_SOFTBODY_FORCE <= force)
		    color = black;
                  b3Printf("force = %f[N*s/s = N]\n", force);

		  m_guiHelper->getRenderInterface()->drawPoint(pos, color, pointSize);
		  total_impulse += impulse.length();
		  if(max_impulse < impulse.length())
		  {
		    max_impulse = impulse.length();
                    // b3Printf("max_impulse = %f[N*s = kg*m/s]\n", max_impulse);
		    max_force = max_impulse / fixedTimeStep;
                    b3Printf("Force(max_impulse/timeStep) = %f[N*s/s = N]\n", max_force);
		  }
		}
	      }
	    }
	    else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
	    {
	      if (multibodyLinkCol)
	      {
		double multiplier = 0.5;
		multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV, -impulse.length() * multiplier);

		btTransform Col_trans = btMultiBodyLinkCollider::upcast(multibodyLinkCol)->getWorldTransform();
		btVector3 Col_central = Col_trans.getOrigin();

		btScalar pointSize = 30; // 20
		// btVector3 pos2 = c.m_node->m_x;
		btVector3 pos = c.m_c1 + Col_central;
		// btScalar delta_color = impulse.length() / MAX_SOFTBODY_IMPULSE;
		btScalar force = impulse.length() / fixedTimeStep;
		btScalar delta_color = force / MAX_SOFTBODY_FORCE; 
		// btVector3 color = btVector3(1, 1, 1-delta_color);

		// btVector3 color = white;
		btVector3 color = green;
		if( 0.f < force && force < MAX_SOFTBODY_FORCE/3)
		  color = green;
		else if( MAX_SOFTBODY_FORCE/3 <= force && force < 2*MAX_SOFTBODY_FORCE/3)
		   color = yellow;
		else if( 2*MAX_SOFTBODY_FORCE/3<= force && force < MAX_SOFTBODY_FORCE)
		  color = red;
		else if( MAX_SOFTBODY_FORCE <= force)
		  color = black;
                b3Printf("force = %f[N*s/s = N]\n", force);

		// b3Printf("deltaV = %f, %f, %f length = %f\n", deltaV.x(), deltaV.y(), deltaV.z());
		// if(impulse.length())
		{
	          // b3Printf("m_x2(node position) = %f, %f, %f\n", c.m_node->m_x.x(), c.m_node->m_x.y(), c.m_node->m_x.z()); // 衝突Nodeの位置
	          // b3Printf("c.m_c1 = %f, %f, %f\n",c.m_c1.x(), c.m_c1.y(), c.m_c1.z()); // Anchorの位置を返す
		  // b3Printf("deltaV = %f, %f, %f length = %f\n", deltaV[0], deltaV[1], deltaV[2]);
		  // b3Printf("impulse*pliter = %f\n", -impulse.length() * multiplier);
		  // b3Printf("impulse = %f, %f, %f length = %f\n", impulse.x(), impulse.y(), impulse.z(), impulse.length()); 
		  m_guiHelper->getRenderInterface()->drawPoint(pos, color, pointSize);
		  // m_guiHelper->getRenderInterface()->drawPoint(pos2, btVector3(1, 0, 0), pointSize);
		  total_impulse += impulse.length();

		  if(max_impulse < impulse.length())
		  {
		    max_impulse = impulse.length();
                    // b3Printf("max_impulse = %f[N*s = kg*m/s]\n", max_impulse);
		    max_force = force;
                    b3Printf("Force(max_impulse/timeStep) = %f[N*s/s = N]\n", max_force);
		  }
		}
	      }
	    }
	  }
        }
      }
      if(max_total_impulse < total_impulse)
      {
	max_total_impulse = total_impulse;
        // b3Printf("max_total_impulse = %f\n", max_total_impulse);
	max_total_force = max_total_impulse / fixedTimeStep;
        // b3Printf("max_total_force(max_total_impulse/timeStep) = %f\n", max_total_force);
      }
      // b3Printf("total_impulse = %f\n", total_impulse);
    }
  }
}
#endif

class CommonExampleInterface* NursingCreateFunc(struct CommonExampleOptions& options)
{
  current_demo = options.m_option;
  return new Nursing(options.m_guiHelper);
}
