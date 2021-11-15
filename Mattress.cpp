#include "Mattress.h"

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
// #include <CommonInterfaces/CommonRigidBodyBase.h>
#include <CommonInterfaces/CommonMultiBodyBase.h>

// #include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

//#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
//#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

#include <Utils/b3ResourcePath.h>
#include <Bullet3Common/b3FileUtils.h>
#include <Importers/ImportObjDemo/LoadMeshFromObj.h>
#include <OpenGLWindow/GLInstanceGraphicsShape.h>
#include <Utils/b3BulletDefaultFileIO.h>

#include <iostream>
#include <fstream>
#include <string>
using namespace std;


string readFile(const char* filename)
{
    ifstream ifs(filename);
    return string(istreambuf_iterator<char>(ifs),
        istreambuf_iterator<char>());
}

void Mattress::registerModel(Nursing* base)
{
    string ele = readFile("Mattress.1.ele");
    string face = readFile("Mattress.1.face");
    string node = readFile("Mattress.1.node");

    //TRACEDEMO

    //#define MESH_MATCH
#ifdef MESH_MATCH
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(base->m_softBodyWorldInfo, gVerticesBunny,
							 &gIndicesBunny[0][0],
							 BUNNY_NUM_TRIANGLES);
  psb->randomizeConstraints();
  psb->setTotalMass(100, true);
  psb->setPose(false, true);

#else
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(base->m_softBodyWorldInfo,
							    ele.c_str(),
							    0, //face.c_str(),
							    node.c_str(),
							    true /*false*/, true /*true*/, true /*true*/);
  // psb->rotate(btQuaternion(0, SIMD_PI / 2, 0));
  psb->rotate(btQuaternion(0, 0, 0));
  // float scale = 9.9;
  float scale = 0.99;
  psb->scale(btVector3(scale, scale, scale));
  // psb->translate(btVector3(0, 0, 1.5));
  psb->translate(btVector3(0, 0, 1.0));
  psb->setVolumeMass(100);
  base->m_cutting = false;
  psb->getCollisionShape()->setMargin(0.01);
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS  
    + btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::SVSmask
    // + btSoftBody::fCollision::SDF_RS
    // + btSoftBody::fCollision::CL_SELF
    ;

  ///pass zero in generateClusters to create  cluster for each tetrahedron or triangle
  psb->generateClusters(0);
  psb->m_materials[0]->m_kLST=0.99;
#endif

  psb->m_cfg.kDF = 1;
  psb->m_cfg.kMT = 0.05;
  psb->m_cfg.diterations = 100;
  psb->m_cfg.piterations = 100;
  psb->m_cfg.kSRHR_CL = 1;

  btSoftMultiBodyDynamicsWorld* softWorld = base->getSoftDynamicsWorld();
  if(softWorld){
  	softWorld->addSoftBody(psb);
        b3Printf("addSoftBody\n");
  }
}
