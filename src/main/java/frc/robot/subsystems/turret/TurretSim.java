// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

public class TurretSim implements TurretIO {
  /** Creates a new TurretSim. */
  public TurretSim() {}

  @Override
  public double degToMotorRevs(double deg) {
    return 0.0;
  }

  @Override
  public void toDeg(double deg) {}

  public void setFlywheelSpeed(double speed) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
