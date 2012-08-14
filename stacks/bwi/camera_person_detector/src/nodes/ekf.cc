// $Id: kalman_mobile.cpp 5925 2006-03-14 21:23:49Z tdelaet $
// Copyright (C) 2006 Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

/* Demonstration program for the Bayesian Filtering Library.
   Mobile robot localization with respect to wall with different possibilities for filter
*/

#include <ros/ros.h>

#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

int main(int argc, char** argv) {

  // Create the matrices A and B for the linear system model
  Matrix A(2,2);
  A(1,1) = 1.0;
  A(1,2) = 0.0;
  A(2,1) = 0.0;
  A(2,2) = 1.0;
  Matrix B(2,2);
  B(1,1) = cos(0.8);
  B(1,2) = 0.0;
  B(2,1) = sin(0.8);
  B(2,2) = 0.0;

  vector<Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // create gaussian
  ColumnVector sysNoise_Mu(2);
  sysNoise_Mu(1) = MU_SYSTEM_NOISE_X;
  sysNoise_Mu(2) = MU_SYSTEM_NOISE_Y;


  SymmetricMatrix sysNoise_Cov(2);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
  sysNoise_Cov(1,2) = 0.0;
  sysNoise_Cov(2,1) = 0.0;
  sysNoise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;

  Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

  // create the model
  LinearAnalyticConditionalGaussian sys_pdf(AB, system_Uncertainty);
  LinearAnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);


  /*********************************
   * Initialise measurement model *
   ********************************/

  // create matrix H for linear measurement model
  Matrix H(1,2);
  double wall_ct = 2/(sqrt(pow(RICO_WALL,2.0) + 1));
  H = 0.0;
  H(1,1) = wall_ct * RICO_WALL;
  H(1,2) = 0 - wall_ct;

  // Construct the measurement noise (a scalar in this case)
  ColumnVector measNoise_Mu(1);
  measNoise_Mu(1) = MU_MEAS_NOISE;

  SymmetricMatrix measNoise_Cov(1);
  measNoise_Cov(1,1) = SIGMA_MEAS_NOISE;
  Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

  // create the model
  LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
  LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);


  /****************************
   * Linear prior DENSITY     *
   ***************************/
   // Continuous Gaussian prior (for Kalman filters)
  ColumnVector prior_Mu(2);
  prior_Mu(1) = PRIOR_MU_X;
  prior_Mu(2) = PRIOR_MU_Y;
  SymmetricMatrix prior_Cov(2);
  prior_Cov(1,1) = PRIOR_COV_X;
  prior_Cov(1,2) = 0.0;
  prior_Cov(2,1) = 0.0;
  prior_Cov(2,2) = PRIOR_COV_Y;
  Gaussian prior(prior_Mu,prior_Cov);




  /******************************
   * Construction of the Filter *
   ******************************/
  ExtendedKalmanFilter filter(&prior);




  /***************************
   * initialise MOBILE ROBOT *
   **************************/
  // Model of mobile robot in world with one wall
  // The model is used to simultate the distance measurements.
  MobileRobot mobile_robot;
  ColumnVector input(2);
  input(1) = 0.1;
  input(2) = 0.0;

  /*******************
   * ESTIMATION LOOP *
   *******************/
  cout << "MAIN: Starting estimation" << endl;
  unsigned int time_step;
  for (time_step = 0; time_step < NUM_TIME_STEPS-1; time_step++)
    {
      // DO ONE STEP WITH MOBILE ROBOT
      mobile_robot.Move(input);

      // DO ONE MEASUREMENT
      ColumnVector measurement = mobile_robot.Measure();

      // UPDATE FILTER
      filter.Update(&sys_model,input,&meas_model,measurement);
    } // estimation loop



  Pdf<ColumnVector> * posterior = filter.PostGet();
  cout << "After " << time_step+1 << " timesteps " << endl;
  cout << " Posterior Mean = " << endl << posterior->ExpectedValueGet() << endl
       << " Covariance = " << endl << posterior->CovarianceGet() << "" << endl;


  cout << "======================================================" << endl
       << "End of the Kalman filter for mobile robot localisation" << endl
       << "======================================================"
       << endl;


  return 0;
}
