// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.turret.TurretIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAimCommand extends Command {
  private final TurretIO turret;
  private final SwerveDriveIO drive;

  /** Creates a new HoodCommand. */
  public TurretAimCommand(TurretIO turret, SwerveDriveIO drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.drive = drive;
    addRequirements(turret);
  }

  /** Creates a new TurretAimCommand. */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot = drive.getPose(); // i created a robot with Pose2d yippee
    Translation2d getToTarget = getHub().minus(robot.getTranslation());
    double fieldAngle = Math.toDegrees(Math.atan2(getToTarget.getY(), getToTarget.getX()));
    double turretAimAngle = fieldAngle - robot.getRotation().getDegrees();
    turretAimAngle = MathUtil.inputModulus(turretAimAngle, -180, 180);
    double wantedAimAngle =
        MathUtil.clamp(turretAimAngle, -Constants.TURRET_LIMIT_DEG, Constants.TURRET_LIMIT_DEG);
    turret.swivel(wantedAimAngle);
    turret.setHood(Constants.START_HOOD_ANGLE);
    turret.flywheelVelocity(0); // i set this to zero and stuff so nothing currently happens
  }

  private static Translation2d getHub() {
    boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return (red ? FieldConstants.RED_GOAL_CENTER : FieldConstants.BLUE_GOAL_CENTER)
        .getTranslation();
  } // this is meant to change the turret command relative to what alliance the robot is on and

  // stuff

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.flywheelVelocity(0); // nothing will happen anymore and stuff
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}