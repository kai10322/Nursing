#include "BedFrame.h"

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
// #include <CommonInterfaces/CommonRigidBodyBase.h>
#include <CommonInterfaces/CommonMultiBodyBase.h>

#include <Utils/b3ResourcePath.h>
#include <Bullet3Common/b3FileUtils.h>
#include <Importers/ImportObjDemo/LoadMeshFromObj.h>
#include <OpenGLWindow/GLInstanceGraphicsShape.h>
#include <Utils/b3BulletDefaultFileIO.h>

void BedFrame::registerModel(CommonMultiBodyBase* base)
{
	//load our obj mesh
	const char* fileName = "BedFrame.obj";  //sphere8.obj";//sponza_closed.obj";//sphere8.obj";
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

	//------------
	float scale = 10;
	btTriangleMesh* m_TriMesh = new btTriangleMesh();
	for (int i=0;i<glmesh->m_numvertices/3;i++) {
	  m_TriMesh->addTriangle( btVector3(scale*glmesh->m_vertices->at(3*i+0).xyzw[0], scale*glmesh->m_vertices->at(3*i+0).xyzw[1], scale*glmesh->m_vertices->at(3*i+0).xyzw[2]), 
				  btVector3(scale*glmesh->m_vertices->at(3*i+1).xyzw[0], scale*glmesh->m_vertices->at(3*i+1).xyzw[1], scale*glmesh->m_vertices->at(3*i+1).xyzw[2]),
				  btVector3(scale*glmesh->m_vertices->at(3*i+2).xyzw[0], scale*glmesh->m_vertices->at(3*i+2).xyzw[1], scale*glmesh->m_vertices->at(3*i+2).xyzw[2]) );
	}
	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(m_TriMesh, true, true);
	
	base->m_collisionShapes.push_back(shape);
	
	float scaling[4] = { scale, scale, scale, 1 };

	btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
	//	shape->setLocalScaling(localScaling);

	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	float color[4] = { 1, 1, 1, 1 };
	float orn[4] = { 0, 0, 0, 1 };
	float pos[4] = { 0, 0, -1.5, 1 }; // why shoud y value be required?
	btVector3 position(pos[0], pos[1], pos[2]);
	startTransform.setOrigin(position);
	btRigidBody* body = base->createRigidBody(mass, startTransform, shape);

	int shapeId = base->m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
		glmesh->m_numvertices,
		&glmesh->m_indices->at(0),
		glmesh->m_numIndices,
		B3_GL_TRIANGLES, -1);
	shape->setUserIndex(shapeId);
	int renderInstance = base->m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
	body->setUserIndex(renderInstance);
}
