#include "BedFrame.h"

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <CommonInterfaces/CommonRigidBodyBase.h>

#include <Utils/b3ResourcePath.h>
#include <Bullet3Common/b3FileUtils.h>
#include <Importers/ImportObjDemo/LoadMeshFromObj.h>
#include <OpenGLWindow/GLInstanceGraphicsShape.h>
#include <Utils/b3BulletDefaultFileIO.h>

void BedFrame::registerModel(CommonRigidBodyBase* base)
{
	//load our obj mesh
	const char* fileName = "BED.obj";  //sphere8.obj";//sponza_closed.obj";//sphere8.obj";
	char relativeFileName[1024];
	if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
	{
		char pathPrefix[1024];
		b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
	}

	b3BulletDefaultFileIO fileIO;
	GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);
	printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);

	const GLInstanceVertex& v = glmesh->m_vertices->at(0);
	btConvexHullShape* shape = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), glmesh->m_numvertices, sizeof(GLInstanceVertex));

	//	float scaling[4] = { 0.1, 0.1, 0.1, 1 };
	float scaling[4] = { 0.0005, 0.0005, 0.0005, 1 };

	btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
	shape->setLocalScaling(localScaling);

//	if (m_options & OptimizeConvexObj)
	if (true)
	{
		shape->optimizeConvexHull();
	}

//	if (m_options & ComputePolyhedralFeatures)
	if (true)
	{
		shape->initializePolyhedralFeatures();
	}

	//shape->setMargin(0.001);
	base->m_collisionShapes.push_back(shape);

	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	float color[4] = { 1, 1, 1, 1 };
	float orn[4] = { 0, 0, 0, 1 };
	float pos[4] = { 0, 0, 0, 0 };
	btVector3 position(pos[0], pos[1], pos[2]);
	startTransform.setOrigin(position);
	btRigidBody* body = base->createRigidBody(mass, startTransform, shape);

//	bool useConvexHullForRendering = ((m_options & ObjUseConvexHullForRendering) != 0);
	bool useConvexHullForRendering = false;

	if (!useConvexHullForRendering)
	{
		int shapeId = base->m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
			glmesh->m_numvertices,
			&glmesh->m_indices->at(0),
			glmesh->m_numIndices,
			B3_GL_TRIANGLES, -1);
		shape->setUserIndex(shapeId);
		int renderInstance = base->m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
		body->setUserIndex(renderInstance);
	}
}
