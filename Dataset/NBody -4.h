#ifndef NBODY_H_
#define NBODY_H_

#include <iostream>
#include <string>

// Local
#include "VectorMath.h"

// Input
# define BODY_COUNT 4
static Position3D nBodyPosition[BODY_COUNT] = 
{
  { 0.0f, 0.0f, -1000.0f },
  { 0.0f, 200.0f, -1000.0f },
  { -200.0f, 0.0f, -1000.0f },
  { 0.0f, 0.0f, -800.0f },
};
static Velocity3D nBodyVelocity[BODY_COUNT] = 
{
  { 0.0f, 0.0f, 0.0f },
  { -3.0f, -3.0f, -3.0f },
  { 3.0f, 3.0f, 3.0f },
  { 4.0f, -3.0f, 1.0f },
};
static Acceleration3D nBodyAcceleration[BODY_COUNT] = 
{
  { 0.0f, 0.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f },
};
static Mass nBodyMass[BODY_COUNT] = 
{
  1e16f,
  1e1f,
  1e1f,
  2e1f,
};

// Methods
static void updatePhysics(float deltaT);
static void updateAcceleration(int bodyIndex);
static void updateVelocity(int bodyIndex, float deltaT);
static void updatePosition(int bodyIndex, float deltaT);


#endif
