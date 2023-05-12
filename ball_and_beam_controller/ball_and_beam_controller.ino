/*********************************************************
 Base Arduino Code for implementing a Digital Controller
 University of Technology Sydney (UTS)
 Board: Teensy 4.1
 Based: bAC22
 Code:  v2

 Created by Ricardo P. Aguiilera,
            Manh (Danny) Duong Phung,
            Ignacio Torres Herrera

 Amended by Alberic Benjamin,
            Yaroslav Syasegov

 Date: 22/04/2022


 Hardward tools: Extended Teesny Board for 4.1x
 Software tools: It requires the following libraries:
    BasicLinearAlgebra
 *******************************************************/

#include <ArduinoEigen.h>

#include "kalman_filter.hpp"
#include "luenberger_observer.hpp"
#include "state_feedback_controller.hpp"
#include "lqr_controller.hpp"
#include "sliding_mode_controller.hpp"
#include "conversions.h"
#include "controller_state.h"
#include <memory>
#include <iostream>

#define fs 100                  // Sampling frequency [Hz] , Ts = 1/fs [s]
float Ts = 1 / float(fs);       // Sampling Time Ts=1/fs [s] in seconds
int Ts_m = (int)(Ts * 1000000); // Must multiply Ts by 1000000!

// Formatting options for matrices
Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");

// Input Pins
// DO NOT CHANGE!!
#define IN1 A1 // Input 1 Pin
#define IN2 A2 // Input 2 Pin
#define IN3 A5 // Input 3 Pin
#define IN4 A6 // Input 4 Pin

// Output Pins
// DO NOT CHANGE!!
#define OUT1 6 // Output 1 Pin
#define OUT2 7 // Output 2 Pin
#define OUT3 4 // Output 3 Pin
#define OUT4 5 // Output 4 Pin

// uC Inputs
uint16_t in3 = 0; // Board Input 3 (+/-12V)
uint16_t in4 = 0; // Board Input 4 (+/-12V)

// uC Outputs
uint16_t out4 = 0; // Board Output 4 (+/-12V)

// Set Default Resolution of PWM Signal
// ADC and PWM/DAC Resolution
// The Due, Zero and MKR Family boards have 12 - bit ADC capabilities
// that can be accessed by changing the resolution to 12.
// 12 bits will return values from analogRead() between 0 and 4095.
// 11 bits will return values from analogRead() between 0 and 2047.
// 10 bits will return values from analogRead() between 0 and 1023.
// Default resolution if not used is 10bits.
const int res = 12;

const size_t state_dimension = 4U;
const size_t state_dimension_integral = 5U;
const size_t control_dimension = 1U;
const size_t output_dimension = 2U;

// forward_dec vars
Eigen::Matrix<double, output_dimension, 1> z_k;
Eigen::Matrix<double, state_dimension, 1> x_hat_k;

// Controller and Observer class instances
std::unique_ptr<KalmanFilter<state_dimension, control_dimension, output_dimension>> kalman_filter;
std::unique_ptr<LuenbergerObserver<state_dimension, control_dimension, output_dimension>> luenberger_observer;
std::unique_ptr<StateFeedbackController<state_dimension, control_dimension>> state_feedback_controller;
std::unique_ptr<StateFeedbackController<state_dimension_integral, control_dimension>> state_feedback_controller_integral;
std::unique_ptr<LqrController<state_dimension, control_dimension>> lqr_controller;
std::unique_ptr<LqrController<state_dimension_integral, control_dimension>> lqr_controller_integral;
std::unique_ptr<SlidingModeController<state_dimension, control_dimension>> sliding_mode_controller;

ControllerState last_controller_state = STOPPED;
ControllerState current_controller_state = STOPPED;
IntervalTimer myTimer;

// Discrete State Space matrices
Eigen::Matrix<double, state_dimension, state_dimension> A;
Eigen::Matrix<double, state_dimension, control_dimension> B;
Eigen::Matrix<double, state_dimension_integral, state_dimension_integral> A_integral;
Eigen::Matrix<double, state_dimension_integral, control_dimension> B_integral;
Eigen::Matrix<double, output_dimension, state_dimension> C;

// Controller and Observer Config
// State feedback gain
Eigen::Matrix<double, control_dimension, state_dimension> K_SFC;
Eigen::Matrix<double, control_dimension, state_dimension_integral> K_SFC_integral;

Eigen::Matrix<double, state_dimension, output_dimension> L;

// Controller reference
Eigen::Matrix<double, state_dimension, 1>
    x_ref;

Eigen::Matrix<double, state_dimension_integral, 1>
    x_ref_integral;

Eigen::Matrix<double, 1, 1>
    q_k;

Eigen::Matrix<double, control_dimension, 1> u_ref;

// hardware noise to create covariance matrices
const double position_variance = 1.1212 / 100.0;
const double angle_variance = 0.0045;
const double voltage_variance = 1e-6;

Eigen::Matrix<double, state_dimension, state_dimension> kalman_Q;

Eigen::Matrix<double, output_dimension, output_dimension> kalman_R;

// initial conditions
// initial observer covariance
Eigen::Matrix<double, state_dimension, state_dimension> P_0;
//  initial observer estimate
Eigen::Matrix<double, state_dimension, 1> x_hat_0;

// LQR params
Eigen::Matrix<double, state_dimension, state_dimension> lqr_Q;
Eigen::Matrix<double, control_dimension, control_dimension> lqr_R;
Eigen::Matrix<double, state_dimension_integral, state_dimension_integral> lqr_Q_integral;
Eigen::Matrix<double, control_dimension, control_dimension> lqr_R_integral;
const double lqr_max_error = 0.1;
const uint32_t lqr_max_iterations = 600;

// Sliding Mode params
const double gamma_sm = 0.4;
const double k = 0.5;
Eigen::Matrix<double, control_dimension, state_dimension> Cs;

//___________________________________________________________________________
//
//                                  setup
//
//            Complete desired initializations on startup
//___________________________________________________________________________
void setup()
{
  // Initialize Serial Bus
  Serial.begin(115200);
  delay(200);

  A << 1.0000, 0.0100, -0.0004, -0.0000,
      0, 1.0000, -0.0701, -0.0003,
      0, 0, 1.0000, 0.0068,
      0, 0, 0, 0.4298;

  A_integral << 1.0000, 0.0100, -0.0004, -0.0000,
      0, 0, 1.0000, -0.0701, -0.0003,
      0, 0, 0, 1.0000, 0.0068,
      0, 0, 0, 0, 0.4298,
      0, 1.0000, 0, 0, 0, 1.0;

  B << 0.0000,
      -0.0000,
      0.0006,
      0.1406;

  B_integral << -0.0000,
      -0.0000,
      0.0008,
      0.1406,
      0;

  C << 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0;

  K_SFC << -2.2001, -2.2845, 9.2207, -2.6050;
  K_SFC_integral << -30.5746, -14.8756, 45.9055, 3.2502, -0.2177;

  L << 0.4418, 0.0081,
      1.9321, 0.2999,
      -0.3381, -0.1212,
      8.7047, 13.3221;

  x_ref << -0.2,
      0.0,
      0.0,
      0.0;

  x_hat_0 << 0.45,
      0.0,
      0.0,
      0.0;

  kalman_Q = Eigen::MatrixXd::Identity(state_dimension, state_dimension) * 1e5;

  kalman_R << pow(position_variance, 2.0), 0,
      0, pow(angle_variance, 2.0);

  P_0 = Eigen::MatrixXd::Identity(state_dimension, state_dimension);

  lqr_Q = C.transpose() * C;
  lqr_R << 0.5;

  lqr_Q_integral = Eigen::DiagonalMatrix<double, 5>(1, 1, 10, 15, 0.0000001);
  lqr_R_integral << 0.1;

  Cs << -1, -1.5, 5, 1;

  x_hat_k = x_hat_0;
  u_ref << 0.0;

  kalman_filter = std::make_unique<KalmanFilter<state_dimension, control_dimension, output_dimension>>(A, B, C, kalman_Q, kalman_R, x_hat_0, P_0);
  luenberger_observer = std::make_unique<LuenbergerObserver<state_dimension, control_dimension, output_dimension>>(A, B, C, L, x_hat_0);
  state_feedback_controller = std::make_unique<StateFeedbackController<state_dimension, control_dimension>>(K_SFC);
  state_feedback_controller_integral = std::make_unique<StateFeedbackController<state_dimension_integral, control_dimension>>(K_SFC_integral);

  lqr_controller = std::make_unique<LqrController<state_dimension, control_dimension>>(A, B, lqr_Q, lqr_R, lqr_max_error, lqr_max_iterations);
  lqr_controller_integral = std::make_unique<LqrController<state_dimension_integral, control_dimension>>(A_integral, B_integral, lqr_Q_integral, lqr_R_integral, lqr_max_error, lqr_max_iterations);
  sliding_mode_controller = std::make_unique<SlidingModeController<state_dimension, control_dimension>>(A, B, Cs, gamma_sm, k);

  // Initialize I/O pins to measure execution time
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);

  // Initialize Input Pins
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);

  // Initialize Output Pins
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);

  // Set Resolution of ADC and DAC
  analogWriteResolution(res);

  // Set Frequency of PWM for modified DAC (do not change this)
  analogWriteFrequency(OUT1, 100000);
  analogWriteFrequency(OUT2, 100000);
  analogWriteFrequency(OUT3, 100000);
  analogWriteFrequency(OUT4, 100000);
}

//___________________________________________________________________________
//
//                              Controller
//
//            The code inside this section will be run at every Ts
//            This is the timer ISR (Interrupt Service Routine).
//___________________________________________________________________________
void Controller(void)
{

  // The code inside this section will be run at every Ts
  // Start measuring execution time
  digitalWrite(A3, HIGH);
  analogWrite(OUT3, 0);

  // Board Inputs
  in3 = analogRead(IN3); // -12v -> 12v
  in4 = analogRead(IN4); // -12v -> 12v

  // Discrete-time Controller
  // Output Measurement
  z_k = {adcToBallPosition(in4), adcToBeamAngleRads(in3)};

  Eigen::Matrix<double, 5, 1> x_hat_k_integral;
  x_hat_k_integral << x_hat_k, q_k;

  Eigen::Matrix<double, 5, 1> x_ref_integral;
  x_ref_integral << x_ref, 0;

  // Control Algorithim
  // auto u_k = state_feedback_controller->compute_control_input(u_ref, x_ref, x_hat_k);
  // auto u_k = lqr_controller->compute_control_input(u_ref, x_ref, x_hat_k);
  // auto u_k = lqr_controller_integral->compute_control_input(u_ref, x_ref_integral, x_hat_k_integral);
  // auto u_k = state_feedback_controller_integral->compute_control_input(u_ref, x_ref_integral, x_hat_k_integral);
  auto u_k = sliding_mode_controller->compute_control_input(u_ref, x_ref, x_hat_k);

  // saturate control action
  u_k(0, 0) = min(u_k(0, 0), 12.0);
  u_k(0, 0) = max(u_k(0, 0), -12.0);

  // Map Contol Effort to output
  out4 = driveVoltageToDAC(u_k(0, 0));

  // Update state estimate
  // x_hat_k = luenberger_observer->compute_observation(u_k, z_k);
  x_hat_k = kalman_filter->filter(u_k, z_k);

  Eigen::Matrix<double, 1, 1> y_hat_k;
  y_hat_k << (C * x_hat_k)(0, 0);
  Eigen::Matrix<double, 1, 1> y_ref;
  y_ref << (C * x_ref)(0, 0);

  q_k = q_k + y_hat_k - y_ref;

  std::cout << "u_k " << u_k.format(CleanFmt) << " pos & angle " << z_k.transpose().format(CleanFmt) << " x_hat " << x_hat_k.transpose().format(CleanFmt) << std::endl;
  std::cout << "last controller gain " << lqr_controller_integral->get_last_gain_matrix().format(CleanFmt) << std::endl;
  std::cout << "q_K" << q_k.transpose().format(CleanFmt) << std::endl;

  // Board Outputs
  analogWrite(OUT4, out4);

  // Stop measuring calculation time
  analogWrite(OUT3, 4000);
}

//___________________________________________________________________________
//
//                                  loop
//
//              Does nothin - main loop left intentionally empty!
//___________________________________________________________________________
void loop()
{
  if (Serial.available() > 0)
  {
    int user_input = Serial.read();

    switch (user_input)
    {
    case 's':
      // Stop mode
      current_controller_state = STOPPED;
      Serial.println("Switching to Stopped");
      break;
    case 'c':
      // control mode
      current_controller_state = CONTROLLING;
      Serial.println("Switching to Controlling");
      break;
    case 'm':
      // manual mode
      current_controller_state = MANUAL;
      Serial.println("Switching to Manual");
    default:
      Serial.println("input to change mode invalid");
    }
  }
  // flush buffer
  Serial.flush();

  if (current_controller_state != last_controller_state)
  {
    if ((current_controller_state == CONTROLLING) && (last_controller_state != CONTROLLING))
    {
      myTimer.begin(Controller, Ts_m);
    }
    else
    {
      myTimer.end();
      double u_k = 0.0;
      analogWrite(OUT4, driveVoltageToDAC(u_k));
    }
  }

  if (current_controller_state != CONTROLLING)
  {
    in3 = analogRead(IN3); // -12v -> 12v
    in4 = analogRead(IN4); // -12v -> 12v
    double u_k = 0.0;
    analogWrite(OUT4, driveVoltageToDAC(u_k));
    Serial.printf("BALL POSITION %f, BALL_ADC %d, BEAM ANGLE %f, ANGLE ADC %d, DAC OUTPUT %d\n", adcToBallPosition(in4), in4, adcToBeamAngleDegrees(in3), in3, driveVoltageToDAC(u_k));
  }

  last_controller_state = current_controller_state;
}
