package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.turret.TurretIO;

public class ContinuousTurretCommand extends Command {
  TurretIO subsystem;
  SwerveDriveIO drive;

  // Creates a new ContinuousTurretCommand.
  public ContinuousTurretCommand(TurretIO subsystem, SwerveDriveIO drive) {
    this.subsystem = subsystem;
    this.drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // CODE GOES IN EXECUTE
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Pose2d hubPose =
          (alliance.get() == DriverStation.Alliance.Red)
              ? FieldConstants.RED_GOAL_CENTER
              : FieldConstants.BLUE_GOAL_CENTER;
      Pose2d robotPose = drive.getPose();
      subsystem.setTargetTurretPos(
          Math.toDegrees(
                  Math.atan2(
                      -(hubPose.getY() - robotPose.getY()), hubPose.getX() - robotPose.getX()))
              + robotPose.getRotation().getDegrees());
    }
    subsystem.setTargetFlywheeelSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
