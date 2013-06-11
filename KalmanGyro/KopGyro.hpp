/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef GYRO_H_
#define GYRO_H_

#include "SensorBase.h"

class AnalogChannel;
class AnalogModule;

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 * The Gyro class tracks the robots heading based on the starting position. As the robot
 * rotates the new heading is computed by integrating the rate of rotation returned
 * by the sensor. When the class is instantiated, it does a short calibration routine
 * where it samples the gyro while at rest to determine the default offset. This is
 * subtracted from each sample to determine the heading. This gyro class must be used 
 * with a channel that is assigned one of the Analog accumulators from the FPGA. See
 * AnalogChannel for the current accumulator assignments.
 */
class KopGyro : public SensorBase
{
public:
	static const UINT32 kOversampleBits = 10;
	static const UINT32 kAverageBits = 0;
	static constexpr float kSamplesPerSecond = 50.0;
	static constexpr float kCalibrationSampleTime = 5.0;
	static constexpr float kDefaultVoltsPerDegreePerSecond = 0.007;

	KopGyro(UINT8 moduleNumber, UINT32 channel);
	explicit KopGyro(UINT32 channel);
	explicit KopGyro(AnalogChannel *channel);
	explicit KopGyro(AnalogChannel &channel);
	virtual ~KopGyro();
	virtual float GetAngle();
	float GetVoltage(); // Returns voltage after oversampling correction
	void SetSensitivity(float voltsPerDegreePerSecond);
	virtual void Reset();

private:
	void InitGyro();

	AnalogChannel *m_analog;
	float m_voltsPerDegreePerSecond;
	float m_offset;
	bool m_channelAllocated;
};
#endif
