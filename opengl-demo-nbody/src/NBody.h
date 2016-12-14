/*
 * NBody.h
 *
 *  Created on: Aug 10, 2012
 *      Author: Antoine Grondin
 */

#ifndef NBODY_H_
#define NBODY_H_

#include <iostream>
#include <string>
#include <GL/glew.h>

#ifdef __APPLE__
#include <glut/glut.h>
#else
#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#endif

// Project libs
#include <GLTools.h>
#include <GLFrustum.h>
#include <GLMatrixStack.h>
#include <GLGeometryTransform.h>
#include <GLShaderManager.h>
#include <StopWatch.h>
#include <math3d.h>

// Local
#include "Constants.h"
#include "VectorMath.h"

const static bool D = DEBUG;

// Variables
// - Matrices and shaders
static GLShaderManager sShaderManager;
static GLFrustum sViewFrustrum;
static GLMatrixStack sProjectionMatrixStack;
static GLMatrixStack sModelViewMatrixStack;
static GLGeometryTransform sTransformPipeline;

////////////////////////////////////////////////////////////////////////
// MODELS
////////////////////////////////////////////////////////////////////////
static GLFrame sCameraFrame;

const static GLclampf sBackgroundColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
const static M3DVector4f sMainLightPos = { 0.0f, 10.0f, 5.0f, 1.0f };

static GLTriangleBatch sBodyBatch[BODY_COUNT];
static GLFrame sBodyFrames[BODY_COUNT];
const static GLfloat sBodyRadius[BODY_COUNT] = {
   50.0f,
   10.0f,
   15.0f,
   5.0f,
   20.0f,
   13.0f,
};

const static GLclampf sBodyColors[BODY_COUNT][4] = {
   {0.8f, 0.8f, 0.1f, 1.0f},  // Yellow
   {0.5f, 0.5f, 1.0f, 1.0f},  // Blue
   {0.8f, 0.8f, 1.0f, 1.0f},  // Light-blue
   {0.8f, 1.0f, 0.8f, 1.0f},  // Light-green
   {1.0f, 1.0f, 1.0f, 1.0f},  // White
   {0.9f, 0.1f, 0.2f, 1.0f}   // Dark red
};

static Position3D sBodyPosition[BODY_COUNT] = {
   { 0.0f, 0.0f, -1000.0f },
   { 0.0f, 200.0f, -1000.0f },
   { -200.0f, 0.0f, -1000.0f },
   { 0.0f, 0.0f, -800.0f },
   { -100.0f, 0.0f, -2000.0f },
   { 100.0f, 0.0f, -500.0f }
};
static Velocity3D sBodyVelocity[BODY_COUNT] = {
   { 0.0f, 0.0f, 0.0f },
   { -30.0f, -30.0f, -30.0f },
   { 30.0f, 30.0f, 30.0f },
   { 45.0f, -30.0f, 15.0f },
   { -30.0f, 20.0f, -45.0f},
   { -30.0f, 20.0f, 10.0f}
};
static Acceleration3D sBodyAcceleration[BODY_COUNT] = {
   { 0.0f, 0.0f, 0.0f },
   { 0.0f, 0.0f, 0.0f },
   { 0.0f, 0.0f, 0.0f },
   { 0.0f, 0.0f, 0.0f },
   { 0.0f, 0.0f, 0.0f },
   { 0.0f, 0.0f, 0.0f }
};
static GLfloat sBodyMass[BODY_COUNT] = {
   1e16f,
   1e1f,
   1e1f,
   2e1f,
   3e1f,
   2e1f
};


///////////////////////////////////////////////////////////////////////
// Methods
///////////////////////////////////////////////////////////////////////

// Setup
void setupWindow( int argc, char **argv );
void registerCallbacks();
void setupRenderContext();
void setupBodies();

// Callbacks
static void onChangeSize( int aNewWidth, int aNewHeight );
static void onRenderScene();


// Drawing
static void drawBodies( CStopWatch *timeKeeper,
                        M3DVector4f *lightPosition );

// Physics
static void updatePhysics( float deltaT );
static void updateAcceleration( int bodyIndex );
static void updateVelocity( int bodyIndex, float deltaT );
static void updatePosition( int bodyIndex, float deltaT );


#endif
