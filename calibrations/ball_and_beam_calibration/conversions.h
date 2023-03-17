#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_
#define ADC_BEAM_ANGLE_CONVERSION_GRADIENT 0.000391
#define ADC_BEAM_ANGLE_CONVERSION_OFFSET  -0.207106
#define ADC_BALL_POSITION_CONVERSION_GRADIENT 0.397018
#define ADC_BALL_POSITION_CONVERSION_OFFSET -201.077739
#define DRIVE_VOLTAGE_DAC_CONVERSION_GRADIENT 0.006148
#define DRIVE_VOLTAGE_DAC_CONVERSION_OFFSET 12.0
#define PI 3.141592

// Beam Angle Conversions
float adcToBeamAngleRads(int adc){
    return (ADC_BEAM_ANGLE_CONVERSION_GRADIENT * adc) + ADC_BEAM_ANGLE_CONVERSION_OFFSET;
}

float adcToBeamAngleDegrees(int adc){
    return adcToBeamAngleRads(adc) * 180.0/PI;
}

// Ball Position Conversion
float adcToBallPosition(int adc){
    return (ADC_BALL_POSITION_CONVERSION_GRADIENT * adc) + ADC_BALL_POSITION_CONVERSION_OFFSET;
}

// Drive Voltage Conversion
uint16_t driveVoltageToDAC(float voltage){
    return (voltage + DRIVE_VOLTAGE_DAC_CONVERSION_OFFSET) / DRIVE_VOLTAGE_DAC_CONVERSION_GRADIENT;
}

#endif