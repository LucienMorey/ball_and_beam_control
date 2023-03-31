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

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#include "kalman_filter.hpp"
#include "state_feedback_controller.hpp"
#include "conversions.h"
#include "controller_state.h"

#define fs 1000                 // Sampling frequency [Hz] , Ts = 1/fs [s]
float Ts = 1 / float(fs);       // Sampling Time Ts=1/fs [s] in seconds
int Ts_m = (int)(Ts * 1000000); // Must multiply Ts by 1000000!

BLA::Matrix<4, 4> EYE_4 = {1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           0.0, 0.0, 0.0, 1.0};

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
 // ADC and PWM/DAC Resolution                                                      \
    The Due, Zero and MKR Family boards have 12-bit ADC capabilities    \
    that can be accessed by changing the resolution to 12.              \
    12 bits will return values from analogRead() between 0 and 4095.    \
    11 bits will return values from analogRead() between 0 and 2047.    \
    10 bits will return values from analogRead() between 0 and 1023.    \
    Default resolution if not used is 10bits.
const int res = 12;

// forward_dec vars
BLA::Matrix<2, 1> z_k;
BLA::Matrix<4, 1> x_hat_k;

// Controller and Observer class instances
KalmanFilter *kalman_filter;
StateFeedbackController *state_feedback_controller;

ControllerState last_controller_state = STOPPED;
ControllerState current_controller_state = STOPPED;
IntervalTimer myTimer;

// System Paramaters
const double length = 0.91;                     // m
const double height = 0.32;                     // M
const double m = 0.03299;                       // kg
const double r = 0.01;                          // m
const double g = 9.81;                          // m/s^2
const double J_b = 2.0 / 5.0 * m * pow(r, 2.0); // kg*m^2
// const double a = -13.3794;                      // TODO FIND THIS
// const double b = 747.4732;                      // TODO FIND THIS
const double k_theta = 0.0; // TODO FIND THIS
const double k_theta_dot = -109.9; //% TODO FIND THIS
const double k_v = 27.8;

// Continuous State Space matrices
BLA::Matrix<4, 4> Ac = {0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, -(m *g) / ((J_b / pow(r, 2.0)) + m), 0.0,
                        0.0, 0.0, 0.0, 1.0,
                        0.0, 0.0, k_theta, k_theta_dot};

BLA::Matrix<4, 1> Bc = {0.0, 0.0, 0.0, k_v};
BLA::Matrix<2, 4> Cc = {1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0};
BLA::Matrix<1, 1> Dc = {0.0};

// Discrete State Space matrices
BLA::Matrix<4, 4> A = EYE_4 - Ac * Ts;
BLA::Matrix<4, 1> B = Bc * Ts;
BLA::Matrix<2, 4> C = Cc;
BLA::Matrix<1, 1> D = Dc;

// Controller and Observer Config
// State feedback gain
BLA::Matrix<1, 4> K_SFC = {
    -1.2794,
    -1.2916,
    5.1679,
    -3.1583,
};

BLA::Matrix<4, 2> L = {
    0.4419,
    0.0050,
    1.9407,
    0.1720,
    -0.5580,
    -0.2177,
    24.8696,
    27.3642,
};

// Controller reference
BLA::Matrix<4, 1> x_ref = {0.0, 0.0, 0.0, 0.0};
double u_ref = 0.0;

// hardware noise to create covariance matrices
const double position_variance = 1.1212 / 100.0;
const double angle_variance = 0.0045;
const double voltage_variance = 1e-6;

BLA::Matrix<4, 4> kalman_Q = EYE_4 * 1e5;
BLA::Matrix<2, 2> kalman_R = {pow(position_variance, 2.0), 0,
                              0, pow(angle_variance, 2.0)};

// initial conditions
// initial observer covariance
BLA::Matrix<4, 4> P_0 = {1.0, 0.0, 0.0, 0.0,
                         0.0, 1.0, 0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 1.0};
//  initial observer estimate
BLA::Matrix<4, 1> x_hat_0 = {0.0, 0.0, 0.0, 0.0};

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

  x_hat_k = x_hat_0;

  kalman_filter = new KalmanFilter(A, B, C, kalman_Q, kalman_R, x_hat_0, P_0);
  state_feedback_controller = new StateFeedbackController(K_SFC);

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
  /*
  Board Inputs
  */
  // read_inputx(disp) //disp = 1 will display input data in Serial Monitor
  in3 = analogRead(IN3); // -12v -> 12v
  in4 = analogRead(IN4); // -12v -> 12v

  // Discrete-time Controller
  // Output Measurement
  z_k = {adcToBallPosition(in4), adcToBeamAngleRads(in3)};

  // Control Algorithim
  double u_k = state_feedback_controller->compute_control_input(u_ref, x_ref, x_hat_k);
  // Serial.printf("voltage %f\n", u_k);
  u_k = min(u_k, 12.0);
  u_k = max(u_k, -12.0);
  // Map Contol Effort to output
  out4 = driveVoltageToDAC(u_k);
  // Update state estimate
  auto y_hat_k = C * x_hat_k;
  x_hat_k = A * x_hat_k + B * u_k + L * (z_k - y_hat_k);

  // debugging prints
  // Serial.printf("BALL POS %f, ANGLE %f\n", adcToBallPosition(in4), adcToBeamAngleDegrees(in3));
  auto last_in = kalman_filter->get_last_innovation();
  // Serial.printf("Last innovation );
  Serial.printf("x_hat_k, %f, %f, %f, %f, u_k %f, position_inno %f, angle inno %f\n",
                x_hat_k(0, 0), x_hat_k(1, 0), x_hat_k(2, 0) * 180 / M_PI,
                x_hat_k(3, 0) * 180 / M_PI, u_k, last_in(0, 0), last_in(1, 0));

  // Serial.printf("Drive Voltage %f\n", u_k);

  /*
  Board Outputs
  */
  // write_outx(value, disp)
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
