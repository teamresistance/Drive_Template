package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TurretReal implements TurretIO {

  // Turret
  TalonFX shooterTurretMotor = new TalonFX(Constants.SHOOTER_TURRET_ID);

  // Flywheeels
  TalonFX shooterFlywheelMotor1 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_1);
  TalonFX shooterFlywheelMotor2 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_2);

  // Creates a new HoodReal.
  public TurretReal() {
    register();

    // PID
    var turretcfg = new TalonFXConfiguration();
    turretcfg.Slot0.kS = 0;
    turretcfg.Slot0.kV = 0;
    turretcfg.Slot0.kP = 0.1;
    turretcfg.Slot0.kI = 0;
    turretcfg.Slot0.kD = 0;
    turretcfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretcfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degsToShooterTurretRevs(135);
    turretcfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretcfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degsToShooterTurretRevs(-135);
    shooterTurretMotor.getConfigurator().apply(turretcfg);
    var flywheel1cfg = new TalonFXConfiguration();
    flywheel1cfg.Slot0.kS = 0;
    flywheel1cfg.Slot0.kV = 0;
    flywheel1cfg.Slot0.kP = 0;
    flywheel1cfg.Slot0.kI = 0;
    flywheel1cfg.Slot0.kD = 0;
    shooterFlywheelMotor1.getConfigurator().apply(flywheel1cfg);
    var flywheel2cfg = new TalonFXConfiguration();
    flywheel2cfg.Slot0.kS = 0;
    flywheel2cfg.Slot0.kV = 0;
    flywheel2cfg.Slot0.kP = 0;
    flywheel2cfg.Slot0.kI = 0;
    flywheel2cfg.Slot0.kD = 0;
    shooterFlywheelMotor2.getConfigurator().apply(flywheel2cfg);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Shooter/Turret/Angle",
        shooterTurretRevsToDegs(shooterTurretMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput(
        "Shooter/Turret/Flywheel Speed", shooterFlywheelMotor1.getVelocity().getValueAsDouble());
  }

  // Sets the shooter hood to the specified angle
  @Override
  public void setTargetTurretPos(double degs) {

    /*
     * If degs is greater than 135, it sets the hood motor to 135 degrees, and if degs is less than
     * -135 degrees, it sets the hood motor to -135 degrees
     */
    double wrapped = MathUtil.inputModulus(degs, -180, 180);
    double clamped = MathUtil.clamp(wrapped, -135, 135);
    Logger.recordOutput("Shooter/Turret/SetpointDeg", clamped);
    shooterTurretMotor.setControl(new PositionVoltage(degsToShooterTurretRevs(clamped)));
  }

  // Sets the flywheels to the desired speed
  @Override
  public void setTargetFlywheeelSpeed(double rpm) {
    shooterFlywheelMotor1.setControl(new VelocityVoltage(rpm));
    shooterFlywheelMotor2.setControl(
        new Follower(Constants.SHOOTER_FLYWHEEL_ID_1, MotorAlignmentValue.Aligned));
  }
}
