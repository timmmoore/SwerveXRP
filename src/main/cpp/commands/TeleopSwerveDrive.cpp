// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopSwerveDrive.h"

#include "subsystems/Drivetrain.h"

TeleopSwerveDrive::TeleopSwerveDrive(
    Drivetrain* subsystem, std::function<double()> xaxisSpeedSupplier,
    std::function<double()> yaxisSpeedSupplier,
    std::function<double()> zaxisRotateSuppplier)
    : m_drive{subsystem},
      m_xaxisSpeedSupplier{xaxisSpeedSupplier},
      m_yaxisSpeedSupplier{yaxisSpeedSupplier},
      m_zaxisRotateSupplier{zaxisRotateSuppplier} {
  AddRequirements(subsystem);
}

void TeleopSwerveDrive::Execute() {
  bool fieldRelative = false;
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_xaxisSpeedSupplier(), 0.2)) *
                        Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_yaxisSpeedSupplier(), 0.2)) *
                        Drivetrain::kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_zaxisRotateSupplier(), 0.2)) *
                     Drivetrain::kMaxAngularSpeed;

  m_drive->Drive(xSpeed, ySpeed, rot, fieldRelative, units::second_t{0.02});
}
