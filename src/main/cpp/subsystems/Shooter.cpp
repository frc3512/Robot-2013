// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Shooter.hpp"

Shooter::Shooter() {
    m_motor1.SetInverted(true);
    m_motor2.SetInverted(true);
    m_controller.SetTolerance(100.0);
}

void Shooter::Enable() { m_enabled = true; }

void Shooter::Disable() { m_enabled = false; }

bool Shooter::IsEnabled() const { return m_enabled; }

void Shooter::SetReference(units::revolutions_per_minute_t angularVelocity) {
    m_controller.SetSetpoint(angularVelocity.to<double>());
}

bool Shooter::AtReference() const { return m_controller.AtSetpoint(); }

void Shooter::Update() {
    if (m_enabled) {
        units::revolutions_per_minute_t speed{m_encoder.GetRate()};
        double feedforward =
            units::revolutions_per_minute_t{m_controller.GetSetpoint()} /
            kMaxSpeed;
        double output =
            m_controller.Calculate(speed.to<double>()) + feedforward;
        m_motor1.Set(output);
        m_motor2.Set(output);
    } else {
        m_motor1.Set(0.0);
        m_motor2.Set(0.0);
    }
}
