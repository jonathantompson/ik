//
//  test_inverse_kinematics.cpp
//
//  Created by Jonathan Tompson on 7/15/14.
//
//  Test inverse kinematics code.  Here a simple 2-chain IK is simulated using
//  PSO as the optimizer.  Note that you can just as easily use BFGS or
//  PSOParallel (the API is more or less inter-changeable).  
//  Levenberg-Marquardt is problably not useful in this context.
//
//  For 
//

#include "test_unit/test_unit.h"
#include "test_unit/test_util.h"
#include "ik/math/pso.h"
#include "ik/math/pso_parallel.h"
#include "ik/math/bfgs.h"
#include "ik/math/lm_fit.h"
#include "ik/clk/clk.h"
#include "ik/math/perlin_noise.h"

#include <thread>
#include <stdlib.h>
#include <GL/freeglut.h>

// Some lazy globals
#define NUM_POSE_COEFFS 6  // 2 joints with 3 degrees of freedom
ik::math::Float3 target_pos(-1,-1,-1);  // End effector target
const float bone_length1 = 0.4;
const float bone_length2 = 0.4;
float cur_coeffs[NUM_POSE_COEFFS];
float start_coeff[NUM_POSE_COEFFS];
float search_radius[NUM_POSE_COEFFS];  // For PSO (not that sensitive)
bool angle_coeff[NUM_POSE_COEFFS];
ik::math::PSO* solver;
ik::clk::Clk clk;

namespace ik {
namespace math {
  void calculateBonePositions(const float* coeff, Float3& bone1, Float3& bone2) {
    // This is lazy, but throw the matrices on the stack (ideally, we would
    // preallocate all this shit)
    Float4x4 mat1;
    Float4x4 mat2;

    // Bugger gimbal lock for now...  Just define DOF as euler angles.
    // We would need to parameterize this properly for real-world code.
    Float4x4::euler2RotMat(mat1, coeff[0], coeff[1], coeff[2]);
    mat1.rightMultTranslation(ik::math::Float3(0, 0, bone_length1));

    Float4x4::euler2RotMat(mat2, coeff[3], coeff[4], coeff[5]);
    mat2.rightMultTranslation(ik::math::Float3(0, 0, bone_length2));

    Float4x4 mat_composite;
    Float4x4::mult(mat_composite, mat1, mat2);

    const Float3 origin(0, 0, 0);
    Float3::affineTransformPos(bone1, mat1, origin);
    Float3::affineTransformPos(bone2, mat_composite, origin);
  }

  void ikCoeffUpdateFunc(float* coeff) { 
    // Nothing to do for this example.  If you wanted to normalize coefficients
    // or apply relaxation methods, this is where you would do it.
  }

  float ikObjectiveFunc(const float* coeff) {
    Float3 bone1, bone2;
    calculateBonePositions(coeff, bone1, bone2);

    // Calculate the L2 distance squared between the bone_pos and the
    // target end effector
    Float3 delta;
    Float3::sub(delta, bone2, target_pos);  // delta = bone2 - target_pos
    float dist_sq = Float3::dot(delta, delta);

    return dist_sq;
  }
  

}  // namespace math
}  // namespace ik

void renderScene(void) {
  // Initialize using the last frame's coefficients
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    start_coeff[i] = cur_coeffs[i];
  }

  // Use perlin noise to animate the end effector
  target_pos[0] = ik::math::PerlinNoise::NoiseNormalized(0, (float)clk.getTime()) * 2 - 1;
  target_pos[1] = ik::math::PerlinNoise::NoiseNormalized(1, (float)clk.getTime()) * 2 - 1;
  target_pos[2] = ik::math::PerlinNoise::NoiseNormalized(2, (float)clk.getTime()) * 2 - 1;

  // Solve the current frame's IK
  ik::math::Float3 bone_pos1;
  ik::math::Float3 bone_pos2;
  solver->minimize(cur_coeffs, start_coeff, search_radius, angle_coeff, 
    ik::math::ikObjectiveFunc, ik::math::ikCoeffUpdateFunc);
  ik::math::calculateBonePositions(cur_coeffs, bone_pos1, bone_pos2);

  // Render the joints
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();
	// Set the camera
	gluLookAt(	0.0f, 0.0f,  3.0f,
			        0.0f, 0.0f,  0.0f,
			        0.0f, 1.0f,  0.0f );

 glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(bone_pos1[0], bone_pos1[1], bone_pos1[2]);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(bone_pos1[0], bone_pos1[1], bone_pos1[2]);
		glVertex3f(bone_pos2[0], bone_pos2[1], bone_pos2[2]);
	glEnd();

  glPointSize(5.0f);
  glBegin(GL_POINTS);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(target_pos[0], target_pos[1], target_pos[2]);

  glEnd();

 glutSwapBuffers();
}

void changeSize(int w, int h) {
	if (h == 0) {
		h = 1;
  }
	float ratio =  w * 1.0f / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45.0f, ratio, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
}

TEST(IK, MSE_2CHAIN) {
  const int swarm_size = 30;
  solver = new ik::math::PSO(NUM_POSE_COEFFS, swarm_size);
  solver->max_iterations = 1000;  // Should be long enough
  solver->delta_coeff_termination = 1e-8f;  // Escentially force max_iterations
  
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    search_radius[i] = 0.1f * (float)M_PI;
  }
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    angle_coeff[i] = true;  // All our coefficients are angles
  }

  // init GLUT and create Window
  int argc = 0;
  char ** argv = NULL;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(640,640);
	glutCreateWindow("2-chain IK example");
	glutDisplayFunc(renderScene);
  glutReshapeFunc(changeSize);
  glutIdleFunc(renderScene);

	// Note: glutSetOption is only available with freeglut
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,
                GLUT_ACTION_GLUTMAINLOOP_RETURNS);

  glutMainLoop();

  delete solver;
}