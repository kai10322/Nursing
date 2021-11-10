#pragma once
// #include <CommonInterfaces/CommonRigidBodyBase.h>
#include <CommonInterfaces/CommonMultiBodyBase.h>

class BedFrame // : public CommonMultiBodyBase
{
	int id;
public:
	static void registerModel(CommonMultiBodyBase* base);
};

