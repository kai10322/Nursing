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
  // psb->setTotalMass(100, true);
  psb->setTotalMass(400, true);
  psb->setPose(false, true);

#else
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(base->m_softBodyWorldInfo,
							    ele.c_str(),
							    0, // face.c_str()
							    node.c_str(),
							    true /*false*/, true /*true*/, true /*true*/);
  // psb->rotate(btQuaternion(0, 0, 0));
  psb->rotate(btQuaternion(0, 0, 0));
  float scale = 1.0;
  // float scale = 0.88;
  psb->scale(btVector3(scale*1.1, scale*1.1, scale*0.6));
  // psb->translate(btVector3(0, 0, 1.0));
  // psb->translate(btVector3(0, 0, -0.25)); // BedFrameがない時
  psb->translate(btVector3(0, 0, 0.0));
  // psb->setVolumeMass(500);
  psb->setVolumeMass(3000);
  // psb->setTotalMass(100);
  base->m_cutting = false;
  // psb->getCollisionShape()->setMargin(0.1);
  psb->getCollisionShape()->setMargin(0.1);
  psb->m_cfg.collisions = // btSoftBody::fCollision::CL_SS  
    // + btSoftBody::fCollision::CL_RS  // 衝突をクラスターで計算
    // + btSoftBody::fCollision::SVSmask
    btSoftBody::fCollision::SDF_RS
    // + btSoftBody::fCollision::CL_SELF
    // + btSoftBody::fCollision::SDF_RD
    + btSoftBody::fCollision::SDF_MDF
    // + btSoftBody::fCollision::Default
    ;
#endif

#if 0  // CL_RS
  // psb->translate(btVector3(0, 0, 0.2));
  // psb->setVolumeMass(500);

  ///pass zero in generateClusters to create  cluster for each tetrahedron or triangle
  psb->generateClusters(16);
  // append Material
  btSoftBody::Material* pm = psb->appendMaterial();
  // pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->m_materials[0]->m_kLST = 1.0;
  // psb->m_materials[0]->m_kAST = 0.8; // 回転に対する剛性
  psb->generateBendingConstraints(3, pm);

  psb->m_cfg.kDF = 1.0; // 動摩擦係数
  // psb->m_cfg.kMT = 1.0; // 元の形状を保とうとする力を働かせる
  // psb->m_cfg.kVC = 1.0; // 体積維持係数
  // psb->m_cfg.kDP = 1.0; // 速度減衰係数
  
  psb->m_cfg.kSR_SPLT_CL = 0.3; // Soft vs rigid impulse split (cluster only)
  psb->m_cfg.diterations = 5;
  psb->m_cfg.piterations = 5;
  psb->m_cfg.citerations = 5;
  psb->m_cfg.kSRHR_CL = 1.0;	// cluster計算時のRigid,SoftのHardress
  psb->m_cfg.kSKHR_CL = 1.0;
  // psb->m_cfg.kSSHR_CL = 1.0;
#endif

#if 1  // SDF_RS
  // psb->translate(btVector3(0, 0, 0.2));
  // psb->setVolumeMass(1000);
  // psb->getCollisionShape()->setMargin(0.1);

  // append Material
  // btSoftBody::Material* pm = psb->appendMaterial();
  psb->m_materials[0]->m_kLST = 0.2;
  psb->m_materials[0]->m_kVST = 0.8;
  psb->m_materials[0]->m_kAST = 0.3;
  psb->generateBendingConstraints(3);
  psb->m_cfg.kDF = 1.0; // 動摩擦係数
  psb->m_cfg.kMT = 0.2; // 元の形状を保とうとする力を働かせる // 0.0
  psb->m_cfg.kVC = 10.0; // 体積維持係数(体積一定にする力)
  // psb->m_cfg.kDP = 1.0; // 速度減衰係数
  psb->m_cfg.viterations = 10;
  psb->m_cfg.diterations = 10;
  psb->m_cfg.piterations = 10;
  psb->m_cfg.kKHR = 1.0;
  psb->m_cfg.kCHR = 1.0;
  // psb->m_cfg.kDP = 0.2;  // Damping係数(空気抵抗) 0.9
  
  // psb->m_cfg.kPR = 0.0;    // 0.0; // Pressure coefficient [-inf,+inf](膨張する力)
  psb->setPose(true, false);
#endif

  // psb->setCcdSweptSphereRadius(0.025);
  // psb->setCcdMotionThreshold(0.0001);

  btSoftMultiBodyDynamicsWorld* softWorld = base->getSoftDynamicsWorld();
  
  if(softWorld){
        int collisionFilterGroup = int(btBroadphaseProxy::DefaultFilter);
        int collisionFilterMask = int(btBroadphaseProxy::AllFilter);
  	softWorld->addSoftBody(psb, collisionFilterGroup, collisionFilterMask);
        // b3Printf("addSoftBody\n");
  }
}

/*
struct Config
{
 	// eAeroModel::_ aeromodel;    // Aerodynamic model (default: V_Point)
	btScalar kVCF;              // Velocities correction factor (Baumgarte)
	btScalar kDP;               // Damping coefficient [0,1] // 空気抵抗
	btScalar kDG;               // Drag coefficient [0,+inf] // 抗力係数(cf.空気力学)
	btScalar kLF;               // Lift coefficient [0,+inf] // 揚力係数(cf.空気力学)
	btScalar kPR;               // Pressure coefficient [-inf,+inf] // 圧力係数(cf.空気力学)
	btScalar kVC;               // Volume conversation coefficient [0,+inf] // 体積保存のための係数(大きいほど体積保存性が高くなる)
	btScalar kDF;               // Dynamic friction coefficient [0,1] // 動摩擦係数
	btScalar kMT;               // Pose matching coefficient [0,1] // 元の形状を保とうとする力を働かせる
	btScalar kCHR;              // Rigid contacts hardness [0,1] // 剛体接触の堅さ
	btScalar kKHR;              // Kinetic contacts hardness [0,1] // 動体接触の堅さ
	btScalar kSHR;              // Soft contacts hardness [0,1] // 柔軟体接触の堅さ
	btScalar kAHR;              // Anchors hardness [0,1] // アンカーの堅さ
	btScalar kSRHR_CL;          // Soft vs rigid hardness [0,1] (cluster only)
	btScalar kSKHR_CL;          // Soft vs kinetic hardness [0,1] (cluster only)
	btScalar kSSHR_CL;          // Soft vs soft hardness [0,1] (cluster only)
	btScalar kSR_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
	btScalar kSK_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
	btScalar kSS_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
	btScalar maxvolume;         // Maximum volume ratio for pose
	btScalar timescale;         // Time scale
	int viterations;            // Velocities solver iterations // 速度修正(ソルバ)の最大反復計算回数
	int piterations;            // Positions solver iterations // 位置修正(ソルバ)の最大反復計算回数
	int diterations;            // Drift solver iterations // ドリフトソルバ反復計算回数
	int citerations;            // Cluster solver iterations // クラスタソルバ反復計算回数
	int collisions;             // Collisions flags
	tVSolverArray m_vsequence;  // Velocity solvers sequence
	tPSolverArray m_psequence;  // Position solvers sequence
	tPSolverArray m_dsequence;  // Drift solvers sequence
	btScalar drag;              // deformable air drag
	btScalar m_maxStress;       // Maximum principle first Piola stress
	m_materials[0]->m_kLST; // 剛性(Linear Stiffness Coefficient) (変形のしやすさ)
	m_materials[0]->m_kAST; // 回転に対する剛性

};
*/
