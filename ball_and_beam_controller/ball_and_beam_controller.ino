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

IntervalTimer myTimer;
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#define fs 200 // Sampling frequency [Hz] , Ts = 1/fs [s]

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

// Control Effort
float u_k;  // u[k]
float u_k1; // u[k-1]

// Plant Outputs
float y1_k;  // y1[k]
float y1_k1; // y1[k-1]

float y2_k;  // y2[k]
float y2_k1; // y1[k-1]

// Control Error
float e_k;  // e[k]
float e_k1; // e[k-1]

// Controller Variables
float a1;
float a2;
float a3;
float b1;
float b2;
float b3;

float z_k;  // z[k]
float z_k1; // z[k-2]

// uC Inputs
float in1 = 0; // Board Input 1 (0->12V)
float in2 = 0; // Board Input 2 (0->12V)
float in3 = 0; // Board Input 3 (+/-12V)
float in4 = 0; // Board Input 4 (+/-12V)

// uC Outputs
float out1 = 0; // Board Output 1 (0->12V)
float out2 = 0; // Board Output 2 (0->12V)
float out3 = 0; // Board Output 3 (+/-12V)
float out4 = 0; // Board Output 4 (+/-12V)

// Set Default Resolution of PWM Signal
 // ADC and PWM/DAC Resolution                                                      \
    The Due, Zero and MKR Family boards have 12-bit ADC capabilities    \
    that can be accessed by changing the resolution to 12.              \
    12 bits will return values from analogRead() between 0 and 4095.    \
    11 bits will return values from analogRead() between 0 and 2047.    \
    10 bits will return values from analogRead() between 0 and 1023.    \
    Default resolution if not used is 10bits.
const int res = 12;

// Continuous State Space matrices
BLA::Matrix<4, 4> Ac;
BLA::Matrix<1, 4> Bc;
BLA::Matrix<2, 4> Cc;
BLA::Matrix<1, 1> Dc;

// Discrete State Space matrices
BLA::Matrix<4, 4> A;
BLA::Matrix<1, 4> B;
BLA::Matrix<2, 4> C;
BLA::Matrix<1, 1> D;

// System Paramaters
const double length = 0.91;                     // m
const double height = 0.32;                     // M
const double m = 0.03299;                       // kg
const double r = 0.01;                          // m
const double g = 9.81;                          // m/s^2
const double J_b = 2.0 / 5.0 * m * pow(r, 2.0); // kg*m^2
const double a = -13.3794;                      // TODO FIND THIS
const double b = 747.4732;                      // TODO FIND THIS

const double position_noise = 0.1;
const double angle_noise = 0.1;
const double voltage_noise = 1e-6;

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

  // Sampling Time Ts=1/fs [s]
  float Ts = 1 / float(fs); // in seconds

  // Initialize controller variables
  a1 = 0;
  a2 = 0;
  a3 = 0;
  b1 = 0;
  b2 = 0;
  b3 = 0;

  // Set up continuous time matrices
  Ac = {0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, -(m * g) / ((J_b / pow(r, 2.0)) + m), 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, a};

  Bc = {0.0, 0.0, 0.0, b};

  Cc = {1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0};

  Dc = {0.0};

  // Discretise continuous time matrices
  BLA::Matrix<4, 4> eye_4 = {1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
  A = eye_4 - Ac * Ts;
  B = Bc * Ts;
  C = Cc;
  D = Dc;

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

  // Create an IntervalTimer object
  int Ts_m = (int)(Ts * 1000000); // Must multiply Ts by 1000000!
  myTimer.begin(Controller, Ts_m);
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

  /*
  Board Inputs
  */

  // read_inputx(disp) //disp = 1 will display input data in Serial Monitor
  in1 = read_input1(0); // 0 -> 12v
  in2 = read_input2(0); // 0 -> 12v
  in3 = read_input3(0); // -12v -> 12v
  in4 = read_input4(0); // -12v -> 12v

  disp_inputs_all(); // Displays all inputs

  // Only display inputs for calibration.
  // Do not display them when running the controller

  // Discrete-time Controller

  /* EXAMPLE - YOU MUST CHANGE */

  // Output Measurement
  y1_k = in1;
  y2_k = in2;

  // Control Algorithim

  u_k = a1 * y1_k - b2 * y2_k1;

  // Map Contol Effort to output

  out1 = u_k;

  // Store variables for next iteration
  y1_k1 = y1_k;

  /*
  Board Outputs
  */
  // disp = 1 will display
  // individual output in serial monitor

  // write_outx(value, disp)
  write_out1(out1, 0);
  write_out2(out2, 0);
  write_out3(out3, 0);
  write_out4(out4, 0);
  // disp_outputs_all();
  // Only display outputs for calibration.
  // Do not display them when running the controller

  // Stop measuring calculation time
  digitalWrite(A3, LOW);
}

//___________________________________________________________________________
//
//                                  loop
//
//              Does nothin - main loop left intentionally empty!
//___________________________________________________________________________
void loop() {}

//___________________________________________________________________________
//
//                              disp_inputs_all
//
//                Displays all inputs on serial monitor
//___________________________________________________________________________
void disp_inputs_all()
{
  Serial.print("In1: ");
  Serial.print(in1, 3);
  Serial.print(" [V]  ");
  Serial.print("In2: ");
  Serial.print(in2, 3);
  Serial.print(" [V]  ");
  Serial.print("In3: ");
  Serial.print(in3, 3);
  Serial.print(" [V]  ");
  Serial.print("In4: ");
  Serial.print(in4, 3);
  Serial.println(" [V]");
}

//___________________________________________________________________________
//                              disp_outputs_all
//
//              Displays all outputs on serial monitor
//___________________________________________________________________________
void disp_outputs_all()
{
  Serial.print("Out1: ");
  Serial.print(out1);
  Serial.print(" [V]  ");
  Serial.print("Out2: ");
  Serial.print(out2);
  Serial.print(" [V]  ");
  Serial.print("Out3: ");
  Serial.print(out3);
  Serial.print(" [V]  ");
  Serial.print("Out4: ");
  Serial.print(out4);
  Serial.print(" [V]  ");
}

//___________________________________________________________________________
//                              read_inputx
//
//        Reads respective analog pin and returns an actual voltage.
//        The raw bits must be scaled!
//___________________________________________________________________________

float read_input1(int disp)
{
  float in_float = 0;

  int ADC_raw = analogRead(IN1); // Read input 1 (0 –> 12V)

  in_float = (float)(ADC_raw);

  // convert ADC_raw to voltage
  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  //  acutal voltage = in_float*m +/-
  float p1 = 1;
  float p2 = 0;
  in_float = in_float * p1 + p2;

  if (disp == 1)
  {
    Serial.print("In1: ");
    Serial.print(in_float);
    Serial.println(" [V]");
  }

  return in_float;
}

float read_input2(int disp)
{
  float in_float = 0;

  int ADC_raw = analogRead(IN2); // Read input 2 (0 –> 12V)

  in_float = (float)(ADC_raw);

  // convert ADC_raw to voltage
  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  //  acutal voltage = in_float*m +/-
  float p1 = 1;
  float p2 = 0;
  in_float = in_float * p1 + p2;

  if (disp == 1)
  {
    Serial.print("In1: ");
    Serial.print(in_float);
    Serial.println(" [V]");
  }

  return in_float;
}

float read_input3(int disp)
{
  float in_float = 0;

  int ADC_raw = analogRead(IN3); // Read input 3 (0 -> +/-12)

  in_float = (float)(ADC_raw);

  // convert ADC_raw to voltage
  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  //  acutal voltage = in_float*m +/-
  float p1 = 1;
  float p2 = 0;
  in_float = in_float * p1 + p2;

  if (disp == 1)
  {
    Serial.print("In1: ");
    Serial.print(in_float);
    Serial.println(" [V]");
  }

  return in_float;
}

float read_input4(int disp)
{
  float in_float = 1;

  int ADC_raw = analogRead(IN4); // Read input 4 (0 -> +/-12)

  in_float = (float)(ADC_raw);

  // convert ADC_raw to voltage
  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  //  acutal voltage = in_float*m +/-
  float p1 = 1;
  float p2 = 0;
  in_float = in_float * p1 + p2;

  if (disp == 1)
  {
    Serial.print("In1: ");
    Serial.print(in_float);
    Serial.println(" [V]");
  }

  return in_float;
}

//___________________________________________________________________________
//                              write_outx
//
//        Given a desired output value, it will scale this to an output
//        value for DAC.
//        The raw bits must be scaled!
//___________________________________________________________________________
void write_out1(float out, int disp)
{
  //  Here, you must enter the experimental linear relationship
  //  between duty cycle d and the actual voltage.
  //  d = out*p1 +/- p2
  float d;
  float p1 = 1;
  float p2 = 0;
  d = out * p1 + p2;

  if (d < 0)
  {
    d = 0;
  }
  if (d > 1)
  {
    d = 1;
  }

  // Convert D to A
  int DAC_raw = (int)(d * 4095);

  // Write to output
  analogWrite(OUT1, DAC_raw);

  // Serial Montior/Plotter
  if (disp == 1)
  {
    Serial.print("Out1 : ");
    Serial.print(out, 10);
    Serial.println(" [V]");
  }
}

void write_out2(float out, int disp)
{
  //  Here, you must enter the experimental linear relationship
  //  between duty cycle d and the actual voltage.
  //  d = out*p1 +/- p2
  float d;
  float p1 = 1;
  float p2 = 0;
  d = out * p1 + p2;

  if (d < 0)
  {
    d = 0;
  }
  if (d > 1)
  {
    d = 1;
  }

  // Convert D to A
  int DAC_raw = (int)(d * 4095);

  // Write to output
  analogWrite(OUT2, DAC_raw);

  // Serial Montior/Plotter
  if (disp == 1)
  {
    Serial.print("Out2 : ");
    Serial.print(out);
    Serial.println(" [V]");
  }
}

void write_out3(float out, int disp)
{
  //  Here, you must enter the experimental linear relationship
  //  between duty cycle d and the actual voltage.
  //  d = out*p1 +/- p2
  float d;
  float p1 = 1;
  float p2 = 0;
  d = out * p1 + p2;

  if (d < 0)
  {
    d = 0;
  }
  if (d > 1)
  {
    d = 1;
  }

  // Convert D to A
  int DAC_raw = (int)(d * 4095);

  // Write to output
  analogWrite(OUT3, DAC_raw);

  // Serial Montior/Plotter
  if (disp == 1)
  {
    Serial.print("Out3 : ");
    Serial.print(out);
    Serial.println(" [V]");
  }
}

void write_out4(float out, int disp)
{
  //  Here, you must enter the experimental linear relationship
  //  between duty cycle d and the actual voltage.
  //  d = out*p1 +/- p2
  float d;
  float p1 = 1;
  float p2 = 0;
  d = out * p1 + p2;

  if (d < 0)
  {
    d = 0;
  }
  if (d > 1)
  {
    d = 1;
  }

  // Convert D to A
  int DAC_raw = (int)(d * 4095);

  // Write to output
  analogWrite(OUT4, DAC_raw);

  // Serial Montior/Plotter
  if (disp == 1)
  {
    Serial.print("Out4 : ");
    Serial.print(out);
    Serial.println(" [V]");
  }
}
