// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnTime.h"

void TurnTime::Initialize() {
  m_timer.Start();
  m_drive->Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
}

void TurnTime::Execute() {
  m_drive->Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{m_speed},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
}

void TurnTime::End(bool interrupted) {
  m_drive->Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
  m_timer.Stop();
  m_timer.Reset();
}

bool TurnTime::IsFinished() {
  return m_timer.HasElapsed(m_duration);
}
