//=============================================================================
//File Name: GeartoothEncoder.hpp
//Description: Counts the number of gear teeth which have passed using a Hall's
//             Effect sensor plugged into a DIO channel and returns the RPM.
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef GEARTOOTH_ENCODER_HPP
#define GEARTOOTH_ENCODER_HPP

/* Usage:
 *
 * This class counts the number of gear teeth which have passed using a Counter
 * and Hall's Effect sensor plugged into a DIO channel. It returns the RPM of
 * the shooter wheel given the gear ratio and number of teeth on the gear.
 */

#include <Counter.h>
#include <Notifier.h>
#include <Synchronized.h>
#include "RollingAverage.hpp"

class GeartoothEncoder {
public:
    GeartoothEncoder( UINT32 channel , UINT32 teeth , float gearRatio );
    ~GeartoothEncoder();

    // Start and stop internal counter
    void start();
    void stop();

    void setTeethPerRevolution( UINT32 teeth );

    void setGearRatio( float ratio );

    // Set number of RPM values to average together
    void setAverageSize( UINT32 size );

    // Returns average RPM
    float getRPM();

    // Set sample rate of RPM data collection thread
    void setSampleRate( UINT32 sampleRate );

private:
    // Counts number of pulses from Hall's Effect sensor
    Counter m_counter;

    // Averages values from encoder
    RollingAverage<float> m_rpmAverager;

    /* Represents conversion factor between RPM of gear and RPM of shooter
     * wheel
     */
    float m_gearRatio;

    // Number of teeth per revolution of gear
    UINT32 m_teeth;

    /* ===== Thread variables ===== */
    // Samples values from encoder at given time interval for averaging
    Notifier* m_sampleThread;

    // Used for getting and setting variables between main and sampling thread
    ReentrantSemaphore m_dataMutex;

    /* Function ran by sampling thread; takes pointer to current class
     * instance as first argument
     */
    static void threadFunc( void* object );
    /* ============================ */
};

#endif // GEARTOOTH_ENCODER_HPP
