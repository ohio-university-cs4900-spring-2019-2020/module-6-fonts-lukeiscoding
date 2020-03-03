#pragma once
#pragma once

//PhysX
#include "PxPhysicsAPI.h"
#include "WO.h"

using namespace physx;

namespace Aftr
{
	class PhysXModule {
	public:
		static void init();
		static void addActor(void* pointer, physx::PxActor* actor);

		static PxPhysics* gPhysics;
		static PxScene* scene;

	protected:
		static PxDefaultAllocator gAllocator;
		static PxDefaultErrorCallback gErrorCallback;
		static PxFoundation* gFoundation;
		static PxSceneDesc gSceneDesc;
		static PxDefaultCpuDispatcher* gCpuDispatcher;
	};
}