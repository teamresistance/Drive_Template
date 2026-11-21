// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

public class ShooterSim implements ShooterIO {

  /** Creates a new ThingSubsystem. */
  public ShooterSim() {

    register();
  }

  @SuppressWarnings("unused")
  private double targetDeg = 0;

  @Override
  public void setTarget(double deg) {}

  @Override
  public double getError() {
    return 0.0;
  }

  @Override
  public void startMotor() {}

  @Override
  public void stopMotor() {}

  @Override
  public boolean overheating() {
    return true;
  }

  @Override
  public void periodic() {}
}
