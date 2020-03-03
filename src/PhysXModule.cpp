#include <iostream>
#include "PhysXModule.h"
#include "AftrGlobals.h"

using namespace Aftr;
using namespace physx;

PxDefaultAllocator PhysXModule::gAllocator;
PxDefaultErrorCallback PhysXModule::gErrorCallback;
PxFoundation* PhysXModule::gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
PxPhysics* PhysXModule::gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true);
PxSceneDesc PhysXModule::gSceneDesc(gPhysics->getTolerancesScale());
PxDefaultCpuDispatcher* PhysXModule::gCpuDispatcher = PxDefaultCpuDispatcherCreate(2);
PxScene* PhysXModule::scene;


void PhysXModule::init() {
	gSceneDesc.cpuDispatcher = gCpuDispatcher;
	gSceneDesc.filterShader = PxDefaultSimulationFilterShader;
	std::cout << "PhysX engine creating!!!!" << std::endl;
	scene = gPhysics->createScene(gSceneDesc);
	scene->setFlag(PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);
	if (!scene) {
		std::cout << "Unable to create PhysX engine" << std::endl;
	}
	else {
		std::cout << "PhysX engine initialized!" << std::endl;
		scene->setGravity(PxVec3(0, 0, -1 * GRAVITY));
	}
}

void PhysXModule::addActor(void* pointer, PxActor* actor) {
	std::cout << "PhysX engine try to add Actor!" << std::endl;
	actor->userData = pointer;
	scene->addActor(*actor);
}