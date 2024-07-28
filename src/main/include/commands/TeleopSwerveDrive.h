// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/Drivetrain.h"

class TeleopSwerveDrive
    : public frc2::CommandHelper<frc2::Command, TeleopSwerveDrive> {
 public:
  TeleopSwerveDrive(Drivetrain* subsystem,
                    std::function<double()> xaxisSpeedSupplier,
                    std::function<double()> yaxisSpeedSupplier,
                    std::function<double()> zaxisRotateSupplier);
  void Execute() override;

 private:
  Drivetrain* m_drive;
  std::function<double()> m_xaxisSpeedSupplier;
  std::function<double()> m_yaxisSpeedSupplier;
  std::function<double()> m_zaxisRotateSupplier;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
};
