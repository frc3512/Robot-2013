// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <mutex>

class KalmanFilter {
public:
    KalmanFilter(double Q, double R);

    void update(double input);

    void setQ(double Q);
    void setR(double R);

    // Return the filtered value
    double getEstimate();

    void reset();

private:
    // Used for getting and setting variables between main and sampling thread
    std::mutex m_dataMutex;

    // Running estimate of state calculated by filter
    double m_xHat;

    // Covariance of error
    double m_P;

    // Process noise variance
    double m_Q;

    // Measurement noise variance
    double m_R;

    // Holds dt in update()
    double m_dt;

    // Used to find dt in update()
    double m_lastTime;

    // If true, update() sets current estimate to first measurement
    bool m_firstRun;
};
