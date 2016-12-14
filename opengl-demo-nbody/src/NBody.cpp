/*
 * NBody.cpp
 *
 *  Created on: Aug 10, 2012
 *      Author: Antoine Grondin
 */

#include "NBody.h"
#include "VectorMath.h"

using std::cout;
using std::cerr;
using std::endl;

///////////////////////////////////////////////////////////////////////
// Setup

int main( int argc, char **argv ) {

   setupWindow( argc, argv );
   registerCallbacks();
   // Initialize GLEW
   GLenum anError = glewInit();

   if( anError != 0 ) {
      fprintf( stderr, "GLEW Error: %s\n",
               glewGetErrorString( anError ) );

      if( D ) {
         cerr << " done" << endl;
      }

      return 1;
   }

   setupRenderContext();
   glutMainLoop();
   return 0;
}

void setupWindow( int argc, char **argv ) {
   gltSetWorkingDirectory( argv[0] );
   glutInit( &argc, argv );
   glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
   glutInitWindowSize( APP_WIDTH, APP_HEIGHT );
   glutCreateWindow( APP_NAME );
   glutFullScreen();
}

void registerCallbacks() {
   glutReshapeFunc( onChangeSize );
   glutDisplayFunc( onRenderScene );
}

void setupRenderContext() {
   sShaderManager.InitializeStockShaders();
   glEnable( GL_DEPTH_TEST );
   setupBodies();
   glClearColor( sBackgroundColor[0],
                 sBackgroundColor[1],
                 sBackgroundColor[2],
                 sBackgroundColor[3] );
   glEnable( GL_LINE_SMOOTH );
}

void setupBodies() {
   for( int i = 0; i < BODY_COUNT; i++ ) {
      gltMakeSphere( sBodyBatch[i], sBodyRadius[i], 30, 50 );
      sBodyFrames[i].SetOrigin( sBodyPosition[i].x,
                                sBodyPosition[i].y,
                                sBodyPosition[i].z );
   }
}

///////////////////////////////////////////////////////////////////////
// Callbacks

void onChangeSize( int aNewWidth, int aNewHeight ) {
   glViewport( 0, 0, aNewWidth, aNewHeight );
   sViewFrustrum.SetPerspective( APP_CAMERA_FOV,
                                 float( aNewWidth ) / float( aNewHeight ),
                                 APP_CAMERA_NEAR,
                                 APP_CAMERA_FAR );
   sProjectionMatrixStack.LoadMatrix(
      sViewFrustrum.GetProjectionMatrix() );
   sTransformPipeline.SetMatrixStacks( sModelViewMatrixStack,
                                       sProjectionMatrixStack );
}

void onRenderScene( void ) {
   // Clear the buffer
   glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
   static CStopWatch timeKeeper;
   // Place camera
   M3DMatrix44f mCamera;
   sCameraFrame.GetCameraMatrix( mCamera );
   sModelViewMatrixStack.PushMatrix( mCamera );
   // Transform the light position into eye coordinates
   M3DVector4f lightPos = { sMainLightPos[0],
                            sMainLightPos[1],
                            sMainLightPos[2],
                            sMainLightPos[3]
                          };
   M3DVector4f lightEyePos;
   m3dTransformVector4( lightEyePos, lightPos, mCamera );
   // Call the drawing functions
   drawBodies( &timeKeeper, &lightEyePos );
   // Switch the buffers to bring the drawing on screen
   glutSwapBuffers();
   glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////
// Drawing

void drawBodies( CStopWatch *timeKeeper, M3DVector4f *lightPosition ) {
   // compute displacement and new vectors
   static float previousTime = 0.0f;
   float currentTime = timeKeeper->GetElapsedSeconds();
   updatePhysics( currentTime - previousTime );
   previousTime = currentTime;

   for( int i = 0; i < BODY_COUNT; i++ ) {
      // Save
      sModelViewMatrixStack.PushMatrix();
      // update position with regard to new values
      sBodyFrames[i].SetOrigin( sBodyPosition[i].x,
                                sBodyPosition[i].y,
                                sBodyPosition[i].z );
      // draw
      sModelViewMatrixStack.MultMatrix( sBodyFrames[i] );
      sShaderManager.UseStockShader( GLT_SHADER_POINT_LIGHT_DIFF,
                                     sTransformPipeline.GetModelViewMatrix(),
                                     sTransformPipeline.GetProjectionMatrix(),
                                     lightPosition,
                                     sBodyColors[i] );
      sBodyBatch[i].Draw();
      // Restore
      sModelViewMatrixStack.PopMatrix();
   }
}

///////////////////////////////////////////////////////////////////////
// Physics

void updatePhysics( float deltaT ) {
   for( int i = 0; i < BODY_COUNT; i++ ) {
      updateAcceleration( i );
      updateVelocity( i, deltaT );
      updatePosition( i, deltaT );
   }
}

void updateAcceleration( int bodyIndex ) {
   Force3D netForce = { 0, 0, 0 };

   for( int i = 0; i < BODY_COUNT; i++ ) {
      if( i == bodyIndex ) {
         continue;
      }

      Force3D vectorForceToOther = {0, 0, 0};
      Force scalarForceBetween = forceNewtonianGravity3D(
                                    sBodyMass[bodyIndex],
                                    sBodyMass[i],
                                    sBodyPosition[bodyIndex],
                                    sBodyPosition[i] );
      direction( sBodyPosition[bodyIndex],
                 sBodyPosition[i],
                 vectorForceToOther );
      vectorForceToOther.x *= scalarForceBetween;
      vectorForceToOther.y *= scalarForceBetween;
      vectorForceToOther.z *= scalarForceBetween;
      netForce.x += vectorForceToOther.x;
      netForce.y += vectorForceToOther.y;
      netForce.z += vectorForceToOther.z;
   }

   sBodyAcceleration[bodyIndex] = computeAccel3D( sBodyMass[bodyIndex],
                                  netForce );
}

void updateVelocity( int bodyIndex, float deltaT ) {
   sBodyVelocity[bodyIndex] = computeVelo3D(
                                 sBodyAcceleration[bodyIndex],
                                 sBodyVelocity[bodyIndex],
                                 deltaT );
}

void updatePosition( int bodyIndex, float deltaT ) {
   sBodyPosition[bodyIndex] = computePos3D( sBodyVelocity[bodyIndex],
                              sBodyPosition[bodyIndex],
                              deltaT );
}

