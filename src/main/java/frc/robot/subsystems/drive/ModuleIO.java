package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {
    // Deliberately empty - default implementation for replay mode
  }

  /** Run the drive motor at the specified open loop value. */
  default void setDriveOpenLoop(double output) {
    // Deliberately empty - default implementation for replay mode
  }

  /** Run the turn motor at the specified open loop value. */
  default void setTurnOpenLoop(double output) {
    // Deliberately empty - default implementation for replay mode
  }

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec) {
    // Deliberately empty - default implementation for replay mode
  }

  /** Run the turn motor to the specified rotation. */
  default void setTurnPosition(Rotation2d rotation) {
    // Deliberately empty - default implementation for replay mode
  }

  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }
}
