package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HoodReal implements HoodIO {
  TalonFX shooterHoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID);

  // Creates a new HoodReal.
  public HoodReal() {
    register();

    // PID
    shooterHoodMotor.setPosition(degsToShooterHoodRevs(Constants.SHOOTER_HOOD_MOTOR_START_DEG));
    var hoodcfg = new TalonFXConfiguration();
    hoodcfg.Slot0.kS = 0;
    hoodcfg.Slot0.kV = 0;
    hoodcfg.Slot0.kP = 0;
    hoodcfg.Slot0.kI = 0;
    hoodcfg.Slot0.kD = 0;
    shooterHoodMotor.getConfigurator().apply(hoodcfg);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Shooter/Hood/Angle",
        shooterHoodRevsToDegs(shooterHoodMotor.getPosition().getValueAsDouble()));
  }

  // Sets the shooter hood to the specified angle
  @Override
  public void setTarget(double degs) {

    // If degs is greater than 45, it sets the hood motor to 45 degrees
    shooterHoodMotor.setControl(
        new PositionVoltage(
            degsToShooterHoodRevs(
                (degs > Constants.SHOOTER_HOOD_MOTOR_MAX_DEG)
                    ? Constants.SHOOTER_HOOD_MOTOR_MAX_DEG
                    : degs)));
  }
}
