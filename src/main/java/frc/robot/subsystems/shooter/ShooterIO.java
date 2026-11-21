// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {

  @Override
  void periodic();

  void startMotor();

  void stopMotor();

  void setTarget(double deg);

  double getError();

  boolean overheating();
}
