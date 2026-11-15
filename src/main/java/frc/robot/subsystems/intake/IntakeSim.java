// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeSim implements IntakeIO {
  /** Creates a new IntakeSim. */
  public IntakeSim() {}

  @Override
  public double motorRotToRoller(double motorRot) {
    return 0;
  }

  @Override
  public double slapDegToRot(double deg) {
    return 0;
  }

  @Override
  public void out() {}

  @Override
  public void middle() {}

  @Override
  public void in() {}

  @Override
  public void brake() {}

  @Override
  public boolean withinTolerance(double deg) {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
