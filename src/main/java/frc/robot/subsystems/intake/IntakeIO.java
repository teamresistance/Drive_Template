// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem {

  @Override
  default void periodic() {}

  default double motorRotToRoller(double motorRot) {
    return 0;
  }

  default double slapDegToRot(double deg) {
    return 0;
  }

  default void out() {}

  default void middle() {}

  default void in() {}

  default void brake() {}

  default boolean withinTolerance(double deg) {
    return false;
  }
}
