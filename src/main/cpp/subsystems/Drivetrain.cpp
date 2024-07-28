// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <numbers>

#include "Constants.h"

using namespace DriveConstants;

// The XRP has the left and right motors set to
// PWM channels 0 and 1 respectively
// The XRP has onboard encoders that are hardcoded
// to use DIO pins 4/5 and 6/7 for the left and right
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) {
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, GetRotation2d(m_gyro.GetAngle()))
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(GetRotation2d(m_gyro.GetAngle()),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::Periodic() {
  // This method will be called once per scheduler run.
}

double Drivetrain::GetAccelX() {
  return m_accelerometer.GetX();
}

double Drivetrain::GetAccelY() {
  return m_accelerometer.GetY();
}

double Drivetrain::GetAccelZ() {
  return m_accelerometer.GetZ();
}

double Drivetrain::GetGyroAngleX() {
  return m_gyro.GetAngleX();
}

double Drivetrain::GetGyroAngleY() {
  return m_gyro.GetAngleY();
}

double Drivetrain::GetGyroAngleZ() {
  return m_gyro.GetAngleZ();
}

void Drivetrain::ResetGyro() {
  m_gyro.Reset();
}
