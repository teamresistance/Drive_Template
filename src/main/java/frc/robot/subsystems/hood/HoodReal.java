package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HoodReal implements HoodIO {
  private final TalonFX motor = new TalonFX(Constants.MOTOR_DEVICE_ID);

  public HoodReal() {
    register();
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 0.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Hood/Angle", motor.getPosition().getValueAsDouble() / Constants.SHOOTER_HOOD_REVS_PER_DEG);
  }

  @Override
  public double angle(double deg) {
    return deg * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  @Override
  public void setHood(double deg) {

    if (deg >= 45) {
      motor.setControl(new PositionVoltage(angle(Constants.MAX_SHOOTER_DEG)));
    } else {
      motor.setControl(new PositionVoltage(angle(deg)));
    }
  }
}
