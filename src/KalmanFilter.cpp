// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#include "KalmanFilter.hpp"

#include <Timer.h>

KalmanFilter::KalmanFilter(double Q, double R) : m_Q(Q), m_R(R) { reset(); }

void KalmanFilter::update(double input) {
    m_dataMutex.lock();

    // Get the current dt since the last call to update()
    m_dt = GetTime() - m_lastTime;

    // Initialize estimate to measured value
    if (m_firstRun) {
        m_xHat = input;

        m_firstRun = false;
    }

    // Make a prediction - project the error covariance ahead
    m_P += m_Q * m_dt;

    /* Calculate the Kalman gain
     *   (m_P + m_R) is S, the estimate error
     */
    double K = m_P / (m_P + m_R);

    /* Update estimate with measurement zk (z_measured)
     *   'input' is zk in this equation
     *   (input - m_xHat) is y, the innovation
     */
    m_xHat += K * (input - m_xHat);

    // Correct error covariance
    m_P -= K * m_P;

    // Test for NaN (all comparisons with NaN will be false)
    if ((!(m_xHat > 0.0)) && (!(m_xHat < 0.0))) {
        m_xHat = 0.0;
    }

    // Update the previous time for the next delta
    m_lastTime = GetTime();

    m_dataMutex.unlock();
}

void KalmanFilter::setQ(double Q) {
    m_dataMutex.lock();

    m_Q = Q;

    m_dataMutex.unlock();
}

void KalmanFilter::setR(double R) {
    m_dataMutex.lock();

    m_R = R;

    m_dataMutex.unlock();
}

double KalmanFilter::getEstimate() {
    m_dataMutex.lock();

    double sXHat = m_xHat;

    m_dataMutex.unlock();

    return sXHat;
}

void KalmanFilter::reset() {
    m_dataMutex.lock();

    m_xHat = 0.0;
    m_P = 0.0;
    m_dt = 0.0;
    m_lastTime = GetTime();
    m_firstRun = true;

    m_dataMutex.unlock();
}
