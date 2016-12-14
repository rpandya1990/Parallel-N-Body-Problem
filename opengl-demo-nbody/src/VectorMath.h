#ifndef VECTORMATH_H_
#define VECTORMATH_H_

#include <iostream>

// Physics and math constants
const static double PI = 3.141592653589f;
const static double G = 6.67e-11f;

// Vectors
typedef double Vector;
typedef Vector Force;
typedef Vector Acceleration;
typedef Vector Position;
typedef Vector Velocity;

typedef struct Vector3D {
   Vector x;
   Vector y;
   Vector z;
};
typedef Vector3D Force3D;
typedef Vector3D Acceleration3D;
typedef Vector3D Velocity3D;
typedef Vector3D Position3D;

// Scalar
typedef double Scalar;
typedef Scalar Mass;
typedef Scalar Time;

// Vector operations
inline Scalar magnitude( const Vector3D &aVector ) {
   Scalar squareOfLength = 0.0;
   squareOfLength += aVector.x * aVector.x;
   squareOfLength += aVector.y * aVector.y;
   squareOfLength += aVector.z * aVector.z;
   return sqrt( squareOfLength );
}

inline void normalize( Vector3D &aVector ) {
   Scalar length = magnitude( aVector );
   aVector.x = aVector.x / length;
   aVector.y = aVector.y / length;
   aVector.z = aVector.z / length;
}

inline void invert( Vector3D &aVector ) {
   aVector.x *= -1.0;
   aVector.y *= -1.0;
   aVector.z *= -1.0;
}

inline void direction( const Vector3D &fromVector,
                       const Vector3D &toVector,
                       Vector3D &resultVector ) {
   resultVector.x = toVector.x - fromVector.x;
   resultVector.y = toVector.y - fromVector.y;
   resultVector.z = toVector.z - fromVector.z;
   normalize( resultVector );
}

// Physics operations

inline Force forceNewtonianGravity3D( Mass onMass, Mass becauseOfMass,
                                      Position3D onPosition, Position3D becauseOfPosition ) {
   Scalar deltaX = becauseOfPosition.x - onPosition.x;
   Scalar deltaY = becauseOfPosition.y - onPosition.y;
   Scalar deltaZ = becauseOfPosition.z - onPosition.z;
   Scalar distance = sqrt(
                        deltaX * deltaX
                        + deltaY * deltaY
                        + deltaZ * deltaZ
                     );

   if( distance == 0 ) {
      return 0;
   }

   Force result = G * ( onMass * becauseOfMass ) /
                  ( distance * distance );
   return result;
}

inline Acceleration computeAccel( Mass mass, Force force ) {
   if( force == 0 ) {
      return 0;
   }

   Scalar result = force / mass;
   return result;
}

inline Velocity computeVelo( Acceleration current,
                             Velocity previous,
                             Time deltaT ) {
   return previous + ( current * deltaT );
}

inline Position computePos( Velocity current,
                            Position previous,
                            Time deltaT ) {
   return previous + ( current * deltaT );
}

inline Acceleration3D computeAccel3D( Mass mass, const Force3D &force ) {
   Acceleration3D anAccelVector = {0, 0, 0};
   anAccelVector.x = computeAccel( mass, force.x );
   anAccelVector.y = computeAccel( mass, force.y );
   anAccelVector.z = computeAccel( mass, force.z );
   return anAccelVector;
}

inline Velocity3D computeVelo3D( Acceleration3D &accel,
                                 Velocity3D &prevVelo,
                                 Time deltaT ) {
   Velocity3D aVelocityVector = {0, 0, 0};
   aVelocityVector.x = computeVelo( accel.x, prevVelo.x, deltaT );
   aVelocityVector.y = computeVelo( accel.y, prevVelo.y, deltaT );
   aVelocityVector.z = computeVelo( accel.z, prevVelo.z, deltaT );
   return aVelocityVector;
}

inline Position3D computePos3D( Velocity3D &velo,
                                Position3D &prevPos,
                                Time deltaT ) {
   Position3D anPositionVector = {0, 0, 0};
   anPositionVector.x = computePos( velo.x, prevPos.x, deltaT );
   anPositionVector.y = computePos( velo.y, prevPos.y,  deltaT );
   anPositionVector.z = computePos( velo.z, prevPos.z,  deltaT );
   return anPositionVector;
}

bool testVectorMath( void ) {
   double tolerateError = 0.01;
   Mass firstMass = 6.0;
   Mass secondMass = 4.0;
   Position3D firstPos = {0, 0, 0};
   Position3D secondPos = {0.02, 0, 0};
   double expected = 4.00e-6;
   double result = forceNewtonianGravity3D( firstMass, secondMass,
                   firstPos, secondPos );

   if( fabs( expected - result ) > tolerateError ) {
      return false;
   }

   return true;
}

#endif
