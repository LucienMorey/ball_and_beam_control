#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_
#define ADC_BEAM_ANGLE_CONVERSION_GRADIENT 0.000430
#define ADC_BEAM_ANGLE_CONVERSION_OFFSET  -0.203152
#define ADC_BALL_POSITION_CONVERSION_GRADIENT 0.003720
#define ADC_BALL_POSITION_CONVERSION_OFFSET -1.855653
#define DRIVE_VOLTAGE_DAC_CONVERSION_GRADIENT 0.006148
#define DRIVE_VOLTAGE_DAC_CONVERSION_OFFSET 12.0

// Beam Angle Conversions
float adcToBeamAngleRads(int adc){
    return (ADC_BEAM_ANGLE_CONVERSION_GRADIENT * adc) + ADC_BEAM_ANGLE_CONVERSION_OFFSET;
}

float adcToBeamAngleDegrees(int adc){
    return adcToBeamAngleRads(adc) * 180.0/PI;
}

// Ball Position Conversion
float adcToBallPosition(int adc){
    return ((ADC_BALL_POSITION_CONVERSION_GRADIENT * adc) + ADC_BALL_POSITION_CONVERSION_OFFSET);
}

// Drive Voltage Conversion
uint16_t driveVoltageToDAC(float voltage){
    return (voltage + DRIVE_VOLTAGE_DAC_CONVERSION_OFFSET) / DRIVE_VOLTAGE_DAC_CONVERSION_GRADIENT;
}

#endif