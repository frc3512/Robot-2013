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

#include "GyroFilter.hpp"

#include <cmath>
#include <Notifier.h>
#include <Timer.h>

GyroFilter::GyroFilter( double (GyroBase::*angleFunc)() , double (GyroBase::*rateFunc)() , GyroBase* funcObj ) {
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

    m_dt = 0.f;
    m_lastTime = GetTime();

    m_sampleThread = new Notifier( &GyroFilter::threadFunc , this );
    m_sampleThread->StartPeriodic( 0.01f );
}

GyroFilter::~GyroFilter() {
    // Stop thread before deleting it
    m_sampleThread->Stop();
    delete m_sampleThread;
}

double GyroFilter::getAngle() {
    return m_angle;
}

void GyroFilter::resetAngle( double newAngle ) {
    m_angle = newAngle;
}

double GyroFilter::getRate() {
    return m_rate;
}

void GyroFilter::calcAngle() {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // Modified again by Tyler Veness
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Get the current dt since the last call to calcAngle()
    m_dt = GetTime() - m_lastTime;

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    m_rate = m_rateFunc() - m_bias;
    m_angle = m_angle + m_dt * m_rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    m_P[0][0] += m_dt * (m_dt * m_P[1][1] - m_P[0][1] - m_P[1][0] + m_Q_angle);
    m_P[0][1] -= m_dt * m_P[1][1];
    m_P[1][0] -= m_dt * m_P[1][1];
    m_P[1][1] += m_Q_bias * m_dt;

    /* === Discrete Kalman filter measurement update equations - Measurement Update ("Correct") === */
    // Calculate the innovation covariance
    /* Step 3 */
    m_S = m_P[0][0] + m_R_measure;

    // Calculate the Kalman gain
    /* Step 4 */
    m_K[0] = m_P[0][0] / m_S;
    m_K[1] = m_P[1][0] / m_S;
    /* ======================================================================================== */

    /* === Calculate angle and bias === */
    // Calculate angle difference
    /* Step 5 */
    m_y = m_angleFunc() - m_angle;

    // Update estimate with measurement zk (newAngle)
    /* Step 6 */
    m_angle = m_angle + m_K[0] * m_y;
    m_bias += m_K[1] * m_y;
    /* ================================ */

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    m_P[0][0] -= m_K[0] * m_P[0][0];
    m_P[0][1] -= m_K[0] * m_P[0][1];
    m_P[1][0] -= m_K[1] * m_P[0][0];
    m_P[1][1] -= m_K[1] * m_P[0][1];

    // Update the previous time for the next delta
    m_lastTime = GetTime();
}

void GyroFilter::threadFunc( void* object ) {
    // Calculate the angle using a Kalman filter
    static_cast<GyroFilter*>( object )->calcAngle();
}
