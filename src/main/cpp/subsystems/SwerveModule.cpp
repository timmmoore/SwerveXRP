// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../../include/subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int driveEncoderChannelA,
                           const int driveEncoderChannelB,
                           const int turningEncoderChannel,
                           const bool driveinverted,
                           const bool turninverted)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
      m_turningEncoder(turningEncoderChannel) {

  m_driveMotor.SetInverted(driveinverted);
  m_turningMotor.SetInverted(turninverted);
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
                                     kEncoderResolution);

  // Limit the PID Controller's input range between 0 and 2pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::angle::radian_t{0.0}, units::angle::radian_t{2.0 * std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
          units::radian_t{m_turningEncoder.Get() * 2 * std::numbers::pi}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetDistance()},
          units::radian_t{m_turningEncoder.Get() * 2 * std::numbers::pi}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.Get() * 2 * std::numbers::pi}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetRate(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.Get() * 2 * std::numbers::pi}, state.angle.Radians());

//  const auto turnFeedforward = m_turnFeedforward.Calculate(
//      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} /*+ turnFeedforward*/);
}
