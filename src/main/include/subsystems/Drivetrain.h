// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/xrp/XRPGyro.h>
#include <frc/xrp/XRPMotor.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>

#include "SwerveModule.h"


class Drivetrain : public frc2::SubsystemBase {
 public:
  static constexpr double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0);  // 48.75:1
  static constexpr double kCountsPerMotorShaftRev = 12.0;
  static constexpr double kCountsPerRevolution =
      kCountsPerMotorShaftRev * kGearRatio;  // 585.0
  static constexpr units::meter_t kWheelDiameter = 60_mm;

  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Returns the acceleration along the X-axis, in Gs.
   */
  double GetAccelX();

  /**
   * Returns the acceleration along the Y-axis, in Gs.
   */
  double GetAccelY();

  /**
   * Returns the acceleration along the Z-axis, in Gs.
   */
  double GetAccelZ();

  /**
   * Returns the current angle of the Romi around the X-axis, in degrees.
   */
  double GetGyroAngleX();

  /**
   * Returns the current angle of the Romi around the Y-axis, in degrees.
   */
  double GetGyroAngleY();

  /**
   * Returns the current angle of the Romi around the Z-axis, in degrees.
   */
  double GetGyroAngleZ();

  /**
   * Reset the gyro.
   */
  void ResetGyro();

  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      1.0_mps;  // 1 //3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second

 private:
  frc::Translation2d m_frontLeftLocation{+0.30832_m, +0.15426_m};
  frc::Translation2d m_frontRightLocation{+0.30832_m, -0.15426_m};
  frc::Translation2d m_backLeftLocation{-0.30832_m, +0.15426_m};
  frc::Translation2d m_backRightLocation{-0.30832_m, -0.15426_m};

  SwerveModule m_frontLeft{0, 1, 4, 5, 12, false, false};
  SwerveModule m_frontRight{2, 3, 6, 7, 13, true, false};
  SwerveModule m_backLeft{4, 5, 8, 9, 14, false, false};
  SwerveModule m_backRight{6, 7, 10, 11, 15, true, true};

  frc::Rotation2d GetRotation2d(double angle) const {
  return units::degree_t{-angle};
  }
  frc::XRPGyro m_gyro;
  frc::BuiltInAccelerometer m_accelerometer;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      GetRotation2d(m_gyro.GetAngle()),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
