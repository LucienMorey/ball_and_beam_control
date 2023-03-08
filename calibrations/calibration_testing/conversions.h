#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

// Beam Angle Conversions
float adcToBeamAngleRads(int adc){
    return (0.000462 * adc) - 0.213671;
}

float adcToBeamAngleDegrees(int adc){
    return (0.026443 * adc) - 12.242443;
}

// Ball Positin Conversion
float adcToBallPosition(int adc){
    return (-0.003895 * adc) + 1.966694;
}

// Drive Voltage Conversion
uint16_t driveVoltageToDAC(float voltage){
    return (voltage + 12.132986) / 0.006148;
}

#endif