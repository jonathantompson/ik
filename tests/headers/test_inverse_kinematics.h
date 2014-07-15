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

#include "test_unit/test_unit.h"
#include "test_unit/test_util.h"
#include "ik/math/pso.h"
#include "ik/math/pso_parallel.h"
#include "ik/math/bfgs.h"
#include "ik/math/lm_fit.h"

#define NUM_POSE_COEFFS 6  // 2 joints with 3 degrees of freedom
ik::math::Float3 target_pos(-1,-1,-1);  // End effector target
const float bone_length1 = 0.5;
const float bone_length2 = 0.5;

namespace ik {
namespace math {
  void ikCoeffUpdateFunc(float* coeff) { 
    // Nothing to do for this example.  If you wanted to normalize coefficients
    // or apply relaxation methods, this is where you would do it.
  }

  float ikObjectiveFunc(const float* coeff) {
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
    Float3 bone_pos;
    Float3::affineTransformPos(bone_pos, mat_composite, origin);

    // Calculate the L2 distance squared between the bone_pos and the
    // target end effector
    Float3 delta;
    Float3::sub(delta, bone_pos, target_pos);  // delta = bone_pos - target_pos
    float dist_sq = Float3::dot(delta, delta);

    return dist_sq;
  }
  
}  // namespace math
}  // namespace ik

TEST(IK, MSE_2CHAIN) {
  const int swarm_size = 30;
  ik::math::PSO* solver = new ik::math::PSO(NUM_POSE_COEFFS, swarm_size);
  solver->max_iterations = 10000;  // Should be long enough
  solver->delta_coeff_termination = 1e-8f;  // Escentially force max_iterations

  float ret_coeffs[NUM_POSE_COEFFS];
  float start_coeff[NUM_POSE_COEFFS];
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    start_coeff[i] = 0.0f;
  }
  float search_radius[NUM_POSE_COEFFS];  // For PSO (not that sensitive)
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    search_radius[i] = (float)M_PI;  // All our coefficients are angles, so pi rad is reasonable
  }
  bool angle_coeff[NUM_POSE_COEFFS];
  for (uint32_t i = 0; i < NUM_POSE_COEFFS; i++) {
    angle_coeff[i] = true;  // All our coefficients are angles
  }

  solver->minimize(ret_coeffs, start_coeff, search_radius, angle_coeff, 
    ik::math::ikObjectiveFunc, ik::math::ikCoeffUpdateFunc);

  delete solver;
}