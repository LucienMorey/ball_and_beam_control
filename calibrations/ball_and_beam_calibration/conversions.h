#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

// Beam Angle Conversions
float adcToBeamAngleRads(int adc){
    return (0.000391 * adc) -0.207106;
}

float adcToBeamAngleDegrees(int adc){
    return (0.023211 * adc) -11.159444;
}

// Ball Positin Conversion
float adcToBallPosition(int adc){
    return (0.397018 * adc) -201.077739;
}

// Drive Voltage Conversion
uint16_t driveVoltageToDAC(float voltage){
    return (voltage +12) / 0.006148;
}

#endif