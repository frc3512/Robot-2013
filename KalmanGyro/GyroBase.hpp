//=============================================================================
//File Name: GyroBase.hpp
//Description: Wrapper for a gyro-accelerometer Kalman filter (GyroFilter)
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

/* Usage:
 *
 * This class is a wrapper for a gyro-accelerometer Kalman Filter. Since this
 * class can't possibly know how to interface with every gyro and accelerometer
 * that exists, you'll instead handle your gyro/accelerometer in your own
 * derived class and pass in the required data from them via the virtual
 * functions. This class is designed for use with a digital (not analog) gyro.
 * TODO: Support analog gyros
 *
 * * To use this interface, derive from this class and override all pure
 *   virtual member functions privately.
 *
 * Function clarifications:
 *   * readGyroX() should return the value of the X-axis of the gyro
 *   * readAccelY() should return the value of the Y-axis of the accelerometer
 *   * getGyroZzero() should return the constant representing the zero point of
 *     the Z-axis of the gyro
 *   * getGyroLSBsPerUnit() should return the number of least significant bits
 *     which represent a change in the gyro's value of 1 degree/second. This
 *     can be found in the data sheet for the specific gyro you're using. Some
 *     data sheets may tell you that one LSB is A degrees/second, or A units
 *     per LSB, in which case 1/A, or 1/A LSBs per unit, would be the value to
 *     return from this function.
 *
 * Misc. Help:
 *   * If your accelerometer or gyro doesn't have one or more of the three
 *     axes, don't inherit the functions related to them. Avoid calling
 *     functions returning data for that respective axis, since it will be
 *     garbage.
 */

#ifndef GYRO_BASE_HPP
#define GYRO_BASE_HPP

#include <Notifier.h>
#include <Synchronized.h>
#include "GyroFilter.hpp"

class GyroBase {
public:
    GyroBase();
    virtual ~GyroBase();

    // Return the internally stored values for gyro angles
    double getXangle();
    double getYangle();
    double getZangle();

    // Used to reset starting angle for the respective axis
    void resetXangle( double newAngle );
    void resetYangle( double newAngle );
    void resetZangle( double newAngle );

    // Return the unbiased rate of the respective axis
    double getXrate();
    double getYrate();
    double getZrate();

protected:
    // Return angle calculated with accelerometer data (returns degrees)
    double getAccelXangle();
    double getAccelYangle();
    double getAccelZangle();

    // Return rate calculated with gyro data
    double getGyroXrate();
    double getGyroYrate();
    double getGyroZrate();

private:
    GyroFilter xFilter;
    GyroFilter yFilter;
    GyroFilter zFilter;

    // Read angle change (usually raw data) from gyro
    virtual int readGyroX();
    virtual int readGyroY();
    virtual int readGyroZ();

    // Read acceleration (usually raw data) from accelerometer
    virtual int readAccelX();
    virtual int readAccelY();
    virtual int readAccelZ();

    // Return zero points for gyro
    virtual double getGyroXzero();
    virtual double getGyroYzero();
    virtual double getGyroZzero();

    // Return zero points for accelerometer
    virtual double getAccelXzero();
    virtual double getAccelYzero();
    virtual double getAccelZzero();

    // Return sensitivity of gyro in least significant bits per degree/second
    virtual double getGyroLSBsPerUnit() = 0;
};

#endif // GYRO_BASE_HPP
