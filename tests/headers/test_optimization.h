//
//  test_optimization.cpp
//
//  Created by Jonathan Tompson on 5/25/13.
//
//  Test BFGS, and other optimizers.
//

#include "ik/math/pso.h"
#include "ik/math/pso_parallel.h"
#include "ik/math/bfgs.h"
#include "ik/math/lm_fit.h"
#include "ik/data_str/vector.h"
#include "test_math/optimization_test_functions.h"

TEST(PSO, ExponentialFit) {
  ik::math::PSO* solver = new ik::math::PSO(NUM_COEFFS_EXPONTIAL_FIT, 17);
  solver->max_iterations = 10000;
  solver->delta_coeff_termination = 1e-8f;

  float ret_coeffs[NUM_COEFFS_EXPONTIAL_FIT];
  float c_rad[NUM_COEFFS_EXPONTIAL_FIT] = {2, 2, 2, 2};

  solver->minimize(ret_coeffs, ik::math::c_0_exponential_fit, 
    c_rad, NULL, ik::math::exponentialFit, NULL);

  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    float k = fabsf(ret_coeffs[i] - ik::math::c_answer_exponential_fit[i]);
    EXPECT_TRUE(k < 0.001f);
  }

  delete solver;
}

namespace ik {
namespace math {
  void coeffUpdateFunc(float* coeff) { 

  }

  void exponentialFitParallel(ik::data_str::Vector<float>& residues, 
    ik::data_str::Vector<float*>& coeffs) {
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      residues[i] = exponentialFit(coeffs[i]);
    }
  }
}  // namespace math
}  // namespace ik

TEST(PSOParallel, ExponentialFit) {
  bool angle_coeffs[NUM_COEFFS_EXPONTIAL_FIT];
  memset(angle_coeffs, 0, sizeof(angle_coeffs[0]) * NUM_COEFFS_EXPONTIAL_FIT);
  ik::math::PSOParallel* solver2 = 
    new ik::math::PSOParallel(NUM_COEFFS_EXPONTIAL_FIT, 64);

  float ret_coeffs2[NUM_COEFFS_EXPONTIAL_FIT];
  float c_rad[NUM_COEFFS_EXPONTIAL_FIT] = {2, 2, 2, 2};

  solver2->minimize(ret_coeffs2, ik::math::c_0_exponential_fit, c_rad, 
    angle_coeffs, ik::math::exponentialFitParallel, 
    &ik::math::coeffUpdateFunc);

  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    float k = fabsf(ret_coeffs2[i] - ik::math::c_answer_exponential_fit[i]);
    EXPECT_TRUE(k < 0.001f);
  }

  delete solver2;
}

TEST(BFGS, HW7_Q4A) {
  ik::math::BFGS<float>* solver_bfgs = new ik::math::BFGS<float>(NUM_COEFFS_HW7_4A);
  solver_bfgs->verbose = false;
  solver_bfgs->max_iterations = 1000;
  solver_bfgs->delta_f_term = 1e-12f;
  solver_bfgs->jac_2norm_term = 1e-12f;
  solver_bfgs->delta_x_2norm_term = 1e-12f;

  float ret_coeffs_bfgs[NUM_COEFFS_HW7_4A];

  solver_bfgs->minimize(ret_coeffs_bfgs, ik::math::c_0_hw7_4a, NULL, 
    ik::math::hw7_4a, ik::math::hw7_4a_jacob, NULL);

  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4A; i++) {
    float k = fabsf(ret_coeffs_bfgs[i] - ik::math::c_answer_hw7_4a[i]);
    EXPECT_TRUE(k < 0.00001f);
  }

  float f_bfgs = ik::math::hw7_4a(ret_coeffs_bfgs);
  float f_answer = ik::math::hw7_4a(ik::math::c_answer_hw7_4a);
  EXPECT_TRUE(fabsf(f_bfgs - f_answer) < 0.00001f);

  delete solver_bfgs;
}

TEST(BFGS, HW7_Q4B_FLOAT) {
  ik::math::BFGS<float>* solver_bfgs2 = new ik::math::BFGS<float>(NUM_COEFFS_HW7_4B);
  solver_bfgs2->verbose = false;
  solver_bfgs2->descent_cond = ik::math::SufficientDescentCondition::ARMIJO;
  solver_bfgs2->max_iterations = 1000;
  solver_bfgs2->delta_f_term = 1e-12f;
  solver_bfgs2->jac_2norm_term = 1e-12f;
  solver_bfgs2->delta_x_2norm_term = 1e-12f;
  
  float ret_coeffs_bfgs2[NUM_COEFFS_HW7_4B];
  
  solver_bfgs2->minimize(ret_coeffs_bfgs2, ik::math::c_0_hw7_4b, NULL, 
    ik::math::hw7_4b, ik::math::hw7_4b_jacob, NULL);
  
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4B; i++) {
    float k = fabsf(ret_coeffs_bfgs2[i] - ik::math::c_answer_hw7_4b[i]);
    EXPECT_TRUE(k < 0.00001f);
  }

  float f_bfgs = ik::math::hw7_4b(ret_coeffs_bfgs2);
  float f_answer = ik::math::hw7_4b(ik::math::c_answer_hw7_4b);
  EXPECT_TRUE(fabsf(f_bfgs - f_answer) < 0.00001f);

  delete solver_bfgs2;
}

TEST(BFGS, HW7_Q4B_DOUBLE) {
  ik::math::BFGS<double>* solver_bfgs2 = new ik::math::BFGS<double>(NUM_COEFFS_HW7_4B);
  solver_bfgs2->verbose = false;
  solver_bfgs2->descent_cond = ik::math::SufficientDescentCondition::STRONG_WOLFE;
  solver_bfgs2->max_iterations = 1000;
  solver_bfgs2->delta_f_term = 1e-12;
  solver_bfgs2->jac_2norm_term = 1e-12;
  solver_bfgs2->delta_x_2norm_term = 1e-12;
  
  double ret_coeffs_bfgs2[NUM_COEFFS_HW7_4B];
  
  solver_bfgs2->minimize(ret_coeffs_bfgs2, ik::math::dc_0_hw7_4b, NULL, 
    ik::math::dhw7_4b, ik::math::dhw7_4b_jacob, NULL);
  
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4B; i++) {
    double k = fabs(ret_coeffs_bfgs2[i] - ik::math::dc_answer_hw7_4b[i]);
    EXPECT_TRUE(k < 0.00001);
  }

  double f_bfgs = ik::math::dhw7_4b(ret_coeffs_bfgs2);
  double f_answer = ik::math::dhw7_4b(ik::math::dc_answer_hw7_4b);
  EXPECT_TRUE(fabs(f_bfgs - f_answer) < 0.00001);

  delete solver_bfgs2;
}

TEST(LM_FIT, HW3_3_DOUBLE) {
  ik::math::LMFit<double>* lm_fit = 
    new ik::math::LMFit<double>(C_DIM_HW3_3, X_DIM_HW3_3, NUM_PTS_HW3_3);
  lm_fit->verbose = false;
  lm_fit->delta_c_termination = 1e-16;
  
  double ret_coeffs[C_DIM_HW3_3];
  
  lm_fit->fitModel(ret_coeffs, ik::math::dc_start_hw3_3, 
    ik::math::dy_vals_hw3_3, ik::math::dx_vals_hw_3_3, 
    ik::math::dfunc_hw3_3, ik::math::djacob_hw3_3, NULL, NULL);
  
  for (uint32_t i = 0; i < C_DIM_HW3_3; i++) {
    double k = fabs(ret_coeffs[i] - ik::math::dc_answer_hw3_3[i]);
    EXPECT_TRUE(k < 0.00001);
  }

  delete lm_fit;
}

TEST(LM_FIT, HW3_3_FLOAT) {
  ik::math::LMFit<float>* lm_fit = 
    new ik::math::LMFit<float>(C_DIM_HW3_3, X_DIM_HW3_3, NUM_PTS_HW3_3);
  lm_fit->verbose = false;
  lm_fit->delta_c_termination = 1e-16f;
  
  float ret_coeffs[C_DIM_HW3_3];
  
  lm_fit->fitModel(ret_coeffs, ik::math::c_start_hw3_3, 
    ik::math::y_vals_hw3_3, ik::math::x_vals_hw_3_3, 
    ik::math::func_hw3_3, ik::math::jacob_hw3_3, NULL, NULL);
  
  for (uint32_t i = 0; i < C_DIM_HW3_3; i++) {
    double k = fabs(ret_coeffs[i] - ik::math::c_answer_hw3_3[i]); 
    EXPECT_TRUE(k < 0.00001);
  }

  delete lm_fit;
}