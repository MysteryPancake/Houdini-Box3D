// PhysicsCases.h
//
// The physics test cases, defined in core_physics.cpp, physics_shapes.cpp and
// physics_bodies.cpp and listed in core_physics.cpp's kTests.
#pragma once

// Identity orientation (x, y, z, w) shared by the cases.
inline const float qIdentity[4] = {0, 0, 0, 1};

void testBodyBasics();
void testPointMapping();
void testMoveEvents();
void testStoredPose();
void testHandleBuild();
void testMidSimChanges();
void testConvexShapes();
void testPrimitiveOverrides();
void testHullCache();
void testHullCentroid();
void testBodyExtent();
void testMirroredPacked();
void testEstimatedBytes();
void testMixHash();
void testFastRotation();
void testBodyPropsEquality();
void testBodyPropsSanitize();
void testCcdThreshold();
void testCollisionFiltering();
void testMassSleepKinematic();
void testKinematicBounce();
void testExternalForces();
void testContactRecycling();
void testSurfaceMaterial();
void testWorldSettings();
void testSleepEnable();
void testBulkCreate();
void testLazyPose();
void testStepProfile();
