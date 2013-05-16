/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

/* Usage:
 *
 * This class is an implementation for a gyro-accelerometer Kalman Filter.
 * Since this class can't possibly know how to interface with every gyro and
 * accelerometer that exists, you'll instead handle your gyro/accelerometer in
 * your own derived class and pass in the required data from them via the pure
 * virtual functions.
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
 *     representing a change in the gyro's value of 1 degree/second. This can
 *     be found in the data sheet for the specific gyro you're using.
 *
 * Implementing callCalcAngle():
 *   * When writing the callCalcAngle() function, you only have to call
 *     calcAngle(2)
 *   * Arguments to calcAngle(2):
 *       * 1st: getAccelXangle() or any other axis (like getAccelYangle())
 *       * 2nd: getGyroXrate() or any other axis (like (getGyroYrate())
 *
 * Misc. Help:
 *   * If your accelerometer or gyro doesn't have one or more of the three
 *     axes, create the function in your class and return zero. As long as you
 *     don't use the axis in callCalcAngle(), the value will never be used.
 */

#ifndef FILTER_BASE_HPP
#define FILTER_BASE_HPP

#include <Notifier.h>
#include <Synchronized.h>
#include <functional>

class GyroBase;

class FilterBase {
public:
    FilterBase( double (GyroBase::*angleFunc)() , double (GyroBase::*rateFunc)() , GyroBase* funcObj );
    virtual ~FilterBase();

    // Return the internally stored value for gyro angle
    double getAngle();

    // Used to reset starting angle
    void resetAngle( double newAngle );

    // Return the unbiased rate
    double getRate();

protected:
    void calcAngle();

private:
    /* Kalman filter variables */
    double m_Q_angle; // Process noise variance for the accelerometer
    double m_Q_bias; // Process noise variance for the gyro bias
    double m_R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double m_angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    double m_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    double m_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double m_P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double m_K[2]; // Kalman gain - This is a 2x1 matrix
    double m_y; // Angle difference - 1x1 matrix
    double m_S; // Estimate error - 1x1 matrix

    /* Returns current angle with data from accelerometer
     * The angle returned should be in degrees
     */
    std::function<double()> m_angleFunc;

    /* Returns current angular rate from gyro
     * The rate returned should be in degrees per second
     */
    std::function<double()> m_rateFunc;

    // Used to find dt in threadFunc(1)
    double m_lastTime;

    /* ===== Thread variables ===== */
    // Samples values from gyro at given time interval for filtering
    Notifier* m_sampleThread;

    // Used for getting and setting variables between main and sampling thread
    ReentrantSemaphore m_angleMutex;
    ReentrantSemaphore m_rateMutex;
    ReentrantSemaphore m_Q_angleMutex;
    ReentrantSemaphore m_Q_biasMutex;
    ReentrantSemaphore m_R_measureMutex;

    /* Function rand by sampling thread; takes pointer to current class
     * instance as first argument
     */
    static void threadFunc( void* object );
    /* ============================ */
};

#endif // FILTER_BASE_HPP
