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
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretIO;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretCommand extends Command {
  private static final Translation2d TURRET_OFFSET = new Translation2d(0.0, 0.0);
  private static final double TURRET_ZERO_OFFSET_DEG = -90.0;

  private final TurretIO turret;
  private final Supplier<Pose2d> poseSupplier;

  /** Creates a new TurretCommand. */
  public TurretCommand(TurretIO turret, Supplier<Pose2d> poseSupplier) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();

    Logger.recordOutput("Robot/Pose", robotPose);
    Translation2d turretPos =
        robotPose.getTranslation().plus(TURRET_OFFSET.rotateBy(robotPose.getRotation()));

    Logger.recordOutput("Turret/Pose", turretPos);
    Translation2d toGoal = getGoal().minus(turretPos);
    double fieldAngleDeg = Math.toDegrees(Math.atan2(toGoal.getY(), toGoal.getX()));

    double turretAngleDeg =
        MathUtil.inputModulus(
            fieldAngleDeg - robotPose.getRotation().getDegrees() - TURRET_ZERO_OFFSET_DEG,
            -180.0,
            180.0);

    Logger.recordOutput("Turret/Angle", turretAngleDeg);

    turret.toDeg(turretAngleDeg);

    turret.setFlywheelSpeed(0.0);
  }

  private static Translation2d getGoal() {
    boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return (red ? FieldConstants.RED_GOAL_CENTER : FieldConstants.BLUE_GOAL_CENTER)
        .getTranslation();
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
