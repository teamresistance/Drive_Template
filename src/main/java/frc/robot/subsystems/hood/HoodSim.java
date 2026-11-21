// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

public class HoodSim implements HoodIO {
  /** Creates a new HoodSim. */
  public HoodSim() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public double angle(double deg) {
    return 0.0;
  }

  @Override
  public void setHood(double deg) {}
}
