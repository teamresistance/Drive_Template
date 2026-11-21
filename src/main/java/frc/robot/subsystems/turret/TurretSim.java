// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

public class TurretSim implements TurretIO {
  /** Creates a new HoodSim. */
  public TurretSim() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public double hoodAngle(double deg) {
    return 0.0;
  }

  @Override
  public double turretAngle(double deg) {
    return 0.0;
  }

  @Override
  public void swivel(double deg) {}

  @Override
  public void setHood(double deg) {}

  @Override
  public void flywheelVelocity(double velocity) {}
}
