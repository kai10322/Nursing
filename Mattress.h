#pragma once
// #include <CommonInterfaces/CommonRigidBodyBase.h>
#include <CommonInterfaces/CommonMultiBodyBase.h>
#include "Nursing.h"
class Mattress // : public CommonMultiBodyBase
{
	int id;
public:
	static void registerModel(Nursing* base);
};

