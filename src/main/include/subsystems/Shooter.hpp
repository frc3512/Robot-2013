// Copyright (c) 2013-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Talon.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>

#include "GeartoothEncoder.hpp"

class Shooter {
public:
    static constexpr auto kMaxSpeed = 5000_rpm;

    Shooter();

    /**
     * Enables shooter controller.
     */
    void Enable();

    /**
     * Disables shooter controller.
     */
    void Disable();

    /**
     * Returns true if the shooter controller is enabled.
     */
    bool IsEnabled() const;

    /**
     * Sets the flywheel's reference angular velocity.
     *
     * @param angularVelocity Reference angular velocity.
     */
    void SetReference(units::revolutions_per_minute_t angularVelocity);

    /**
     * Returns true if the controller has reached the reference.
     */
    bool AtReference() const;

    /**
     * Updates motor outputs with controller output.
     */
    void Update();

private:
    frc::Talon m_motor1{9};
    frc::Talon m_motor2{10};
    GeartoothEncoder m_encoder{9, 56, 4.0};
    frc2::PIDController m_controller{0.0015, 0.000096, 0.0, 20_ms};
    bool m_enabled = false;
};
