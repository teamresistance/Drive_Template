// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HoodReal implements HoodIO {
  private TalonFX hoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID);

  /** Creates a new HoodReal. */
  public HoodReal() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 0.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.setPosition(degToMotorRevs(13));
  }

  @Override
  public double degToMotorRevs(double deg) {
    return deg * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  @Override
  public void toDeg(double deg) {
    if (deg >= 45) deg = 45;
    hoodMotor.setControl(new PositionVoltage(degToMotorRevs(deg)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Hood/Angle",
        hoodMotor.getPosition().getValueAsDouble() / Constants.SHOOTER_HOOD_REVS_PER_DEG);
  }
}
