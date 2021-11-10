void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->setGravity( btVector3(0, -1000*scale, 0) );

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);	

	//load polishing bowl
	const char* fileName = "bowl.stl";
	char relativeFileName[1024];
	if (!b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024)) {
		b3Warning("Cannot find file %s\n", fileName);
		return;
	}

	GLInstanceGraphicsShape* glmesh = LoadMeshFromSTL(relativeFileName);
	if (glmesh && (glmesh->m_numvertices>0)) {
		btTriangleMesh* m_TriMesh = new btTriangleMesh();
		for (int i=0;i<glmesh->m_numvertices/3;i++) {
			m_TriMesh->addTriangle( btVector3(scale*glmesh->m_vertices->at(3*i+0).xyzw[0], scale*glmesh->m_vertices->at(3*i+0).xyzw[1], scale*glmesh->m_vertices->at(3*i+0).xyzw[2]), 
									btVector3(scale*glmesh->m_vertices->at(3*i+1).xyzw[0], scale*glmesh->m_vertices->at(3*i+1).xyzw[1], scale*glmesh->m_vertices->at(3*i+1).xyzw[2]),
									btVector3(scale*glmesh->m_vertices->at(3*i+2).xyzw[0], scale*glmesh->m_vertices->at(3*i+2).xyzw[1], scale*glmesh->m_vertices->at(3*i+2).xyzw[2]) );
		}
		btBvhTriangleMeshShape* bowlShape = new btBvhTriangleMeshShape(m_TriMesh, true, true);

		m_collisionShapes.push_back(bowlShape);
		btTransform bowlTransform;
		bowlTransform.setIdentity();
		btScalar mass(0.);
		bowl = createRigidBody(mass,bowlTransform,bowlShape, btVector4(0,0,1,1));
		bowl->setFriction(1.0);
		bowl->setRollingFriction(1.0);
	}

	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance
	btCapsuleShape* colShape = new btCapsuleShape(btScalar(5*scale), btScalar(10*scale));
	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();
	
	btScalar mass(1.0*scale);
	btVector3 localInertia(0,0,0);
	colShape->calculateLocalInertia(mass,localInertia);

	float inRad = 130.0*scale;
	float outRad = 270.0*scale;

	for (int k=0;k<ARRAY_SIZE_Y;k++) {
		for (int i=0;i<ARRAY_SIZE_X;i++) {
			for(int j = 0;j<ARRAY_SIZE_Z;j++) {
				startTransform.setOrigin(btVector3( btScalar( (inRad+(outRad-inRad)*i/ARRAY_SIZE_X) * cos(6.28*j/ARRAY_SIZE_Z) ),
													btScalar( (125+10*k)*scale ),
													btScalar( (inRad+(outRad-inRad)*i/ARRAY_SIZE_X) * sin(6.28*j/ARRAY_SIZE_Z) ) ));
			
				btRigidBody* particle;createRigidBody(mass,startTransform,colShape);					
				particle->setFriction(1.0);
				particle->setRollingFriction(1.0);
			}
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}
