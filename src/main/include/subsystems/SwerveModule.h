// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/xrp/XRPMotor.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/DutyCycleEncoder.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int driveEncoderChannelA, int driveEncoderChannelB,
               int turningEncoderChannel,
               bool driveinverted,
               bool turninverted);
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.02559;
  static constexpr int kEncoderResolution = (28 * 3 * 2 * 30)/12;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 0.5_rad_per_s; //1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 2_rad_per_s / 2_s; //1_s;  // radians per second^2

  frc::XRPMotor m_driveMotor;
  frc::XRPMotor m_turningMotor;

  frc::Encoder m_driveEncoder;
  frc::DutyCycleEncoder m_turningEncoder;

  frc::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      2.5,
      0.1,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1.5_V, 1.0_V / 1_rad_per_s};      // Kv calculated at 8.6
};
