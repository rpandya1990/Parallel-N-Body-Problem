/* Sequential version of N body simulation */
/* Author: Raghav Pandya */

#include "NBody.h"
#include "VectorMath.h"
#include "CycleTimer.h"

using namespace std;

// Compute forces on each body with time step

void display_bodies()
{
  for( int i = 0; i < BODY_COUNT; i++ ) 
  {
    printf("\nBody %d:\nMass: %f\nPosition(x ,y, z): %f, %f, %f\nVelocity(x, y, z): %f, %f, %f\nAcceleration(x ,y, z): %f, %f, %f\n\n",
      i + 1, 
      nBodyMass[i], 
      nBodyPosition[i].x, nBodyPosition[i].y, nBodyPosition[i].z,
      nBodyVelocity[i].x, nBodyVelocity[i].y, nBodyVelocity[i].z,
      nBodyAcceleration[i].x, nBodyAcceleration[i].y, nBodyAcceleration[i].z);
  }
}

void compute() 
{
  // display_bodies();
  double start, end, minSerial = 1e30;
  for (int j = 0; j < 1; ++j)
  {
    start = CycleTimer::currentSeconds(); 
    for (int i = 0; i < 10000; ++i)
     {
       updatePhysics(i * 100);
     }
    end = CycleTimer::currentSeconds();
    minSerial = std::min(minSerial, end - start);
  }
  printf("Time Taken by Serial implementation: %f ms\n", (minSerial)*1000);
}

// Physics

void updateAcceleration(int bodyIndex ) 
{
   
  Force3D netForce = { 0, 0, 0 };

  for( int i = 0; i < BODY_COUNT; i++ ) 
  {
    if( i == bodyIndex ) 
    {
      continue;
    }

    Force3D vectorForceToOther = {0, 0, 0};
    Force scalarForceBetween = forceNewtonianGravity3D(
                                  nBodyMass[bodyIndex],
                                  nBodyMass[i],
                                  nBodyPosition[bodyIndex],
                                  nBodyPosition[i]);
    direction( 
      nBodyPosition[bodyIndex],
      nBodyPosition[i],
      vectorForceToOther);

    vectorForceToOther.x *= scalarForceBetween;
    vectorForceToOther.y *= scalarForceBetween;
    vectorForceToOther.z *= scalarForceBetween;
    netForce.x += vectorForceToOther.x;
    netForce.y += vectorForceToOther.y;
    netForce.z += vectorForceToOther.z;
  }

  nBodyAcceleration[bodyIndex] = computeAccel3D(nBodyMass[bodyIndex], netForce);
}

void updateVelocity( int bodyIndex, float deltaT ) 
{
  nBodyVelocity[bodyIndex] = computeVelo3D(
                                nBodyAcceleration[bodyIndex],
                                nBodyVelocity[bodyIndex],
                                deltaT);
}

void updatePosition( int bodyIndex, float deltaT ) 
{

  nBodyPosition[bodyIndex] = computePos3D( 
                                nBodyVelocity[bodyIndex],
                                nBodyPosition[bodyIndex],
                                deltaT);
}

void updatePhysics(float deltaT)
{
  for( int i = 0; i < BODY_COUNT; i++ ) 
  {
    updateAcceleration(i);
    updateVelocity(i, deltaT );
    updatePosition(i, deltaT );
  }
}

int main() 
{  
  compute();
  return 0;
}
