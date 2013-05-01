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
 * Implementing callCalcAngle(1):
 *   * When writing the callCalcAngle(1) function, you only have to call
 *     calcAngle(3)
 *   * Arguments to calcAngle(3):
 *       * 1st: getAccelXangle() or any other axis (like getAccelYangle())
 *       * 2nd: getGyroXrate() or any other axis (like (getGyroYrate())
 *       * 3rd: the "dt" parameter of callCalcAngle(1)
 *
 * Misc. Help:
 *   * If your accelerometer or gyro doesn't have one or more of the three
 *     axes, create the function in your class and return zero. As long as you
 *     don't use the axis in callCalcAngle(1), the value will never be used.
 */

#ifndef KALMAN_GYRO_HPP
#define KALMAN_GYRO_HPP

#include <Notifier.h>
#include <Synchronized.h>

class KalmanGyro {
public:
    KalmanGyro();
    virtual ~KalmanGyro();

    // Return the internally stored value for gyro angle
    double getAngle();

    // Used to reset starting angle
    void resetAngle( double newAngle );

    // Return the unbiased rate
    double getRate();

    // These are used to tune the Kalman filter
    void setQangle( double newQ_angle );
    void setQbias( double newQ_bias );
    void setRmeasure( double newR_measure );

protected:
    // Return angle calculated with accelerometer data (returns degrees)
    double getAccelXangle();
    double getAccelYangle();
    double getAccelZangle();

    // Return rate calculated with gyro data
    double getGyroXrate();
    double getGyroYrate();
    double getGyroZrate();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    void calcAngle( double newAngle , double newRate , double dt );

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

    // Read angle change (usually raw data) from gyro
    virtual int readGyroX() = 0;
    virtual int readGyroY() = 0;
    virtual int readGyroZ() = 0;

    // Read acceleration (usually raw data) from accelerometer
    virtual int readAccelX() = 0;
    virtual int readAccelY() = 0;
    virtual int readAccelZ() = 0;

    // Return zero points for gyro
    virtual double getGyroXzero() = 0;
    virtual double getGyroYzero() = 0;
    virtual double getGyroZzero() = 0;

    // Return zero points for accelerometer
    virtual double getAccelXzero() = 0;
    virtual double getAccelYzero() = 0;
    virtual double getAccelZzero() = 0;

    // Return sensitivity of gyro in least significant bits per degree/second
    virtual double getGyroLSBsPerUnit() = 0;

    /* The implementing class determines the correct arguments to pass to
     * calcAngle(3) and calls calcAngle(3)
     */
    virtual void callCalcAngle( double dt ) = 0;

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

#endif // KALMAN_GYRO_HPP
