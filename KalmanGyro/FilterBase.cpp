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

#include "FilterBase.hpp"

#include <cmath>
#include <Timer.h>

FilterBase::FilterBase( double (GyroBase::*angleFunc)() , double (GyroBase::*rateFunc)() , GyroBase* funcObj ) {
    /* We will set the variables like so, these can also be tuned by the user */
    m_Q_angle = 0.001;
    m_Q_bias = 0.003;
    m_R_measure = 0.03;

    m_angle = 180.0;
    m_bias = 0.0; // Reset bias
    m_rate = 0.0;

    m_P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    m_P[0][1] = 0;
    m_P[1][0] = 0;
    m_P[1][1] = 0;
    m_y = 0;
    m_S = 0;

    m_angleFunc = std::bind( angleFunc , funcObj );
    m_rateFunc = std::bind( rateFunc , funcObj );

    m_lastTime = GetTime();

    m_sampleThread = new Notifier( &FilterBase::threadFunc , this );
    m_sampleThread->StartPeriodic( 0.01f );
}

FilterBase::~FilterBase() {
    // Stop thread before deleting it
    m_sampleThread->Stop();
    delete m_sampleThread;
}

double FilterBase::getAngle() {
    double sAngle = 0.0;

    m_angleMutex.take();

    sAngle = m_angle;

    m_angleMutex.give();

    return sAngle;
}

void FilterBase::resetAngle( double newAngle ) {
    m_angleMutex.take();

    m_angle = newAngle;

    m_angleMutex.give();
}

double FilterBase::getRate() {
    double sRate = 0.0;

    m_rateMutex.take();

    sRate = m_rate;

    m_rateMutex.give();

    return sRate;
}

void FilterBase::calcAngle() {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Get the current dt since the last call to calcAngle(2)
    double dt = GetTime() - m_lastTime;

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    m_rateMutex.take();
    m_rate = m_rateFunc() - m_bias;
    m_angleMutex.take();
    m_angle += dt * m_rate;
    m_angleMutex.give();
    m_rateMutex.give();

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    m_Q_angleMutex.take();
    m_P[0][0] += dt * (dt * m_P[1][1] - m_P[0][1] - m_P[1][0] + m_Q_angle);
    m_Q_angleMutex.give();
    m_P[0][1] -= dt * m_P[1][1];
    m_P[1][0] -= dt * m_P[1][1];
    m_Q_biasMutex.take();
    m_P[1][1] += m_Q_bias * dt;
    m_Q_biasMutex.give();

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 3 */
    m_R_measureMutex.take();
    m_S = m_P[0][0] + m_R_measure;
    m_R_measureMutex.give();

    /* Step 4 */
    m_K[0] = m_P[0][0] / m_S;
    m_K[1] = m_P[1][0] / m_S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 5 */
    m_angleMutex.take();
    m_y = m_angleFunc() - m_angle;

    /* Step 6 */
    m_angle += m_K[0] * m_y;
    m_angleMutex.give();
    m_bias += m_K[1] * m_y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    m_P[0][0] -= m_K[0] * m_P[0][0];
    m_P[0][1] -= m_K[0] * m_P[0][1];
    m_P[1][0] -= m_K[1] * m_P[0][0];
    m_P[1][1] -= m_K[1] * m_P[0][1];

    // Update the previous time for the next delta
    m_lastTime = GetTime();
}

void FilterBase::threadFunc( void* object ) {
    // Calculate the angle using a Kalman filter
    static_cast<FilterBase*>( object )->calcAngle();
}
