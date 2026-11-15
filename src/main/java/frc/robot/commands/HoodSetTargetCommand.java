package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.hood.HoodIO;

public class HoodSetTargetCommand extends Command {
  HoodIO subsystem;
  double degs;

  // Creates a new HoodSetTargetCommand.
  public HoodSetTargetCommand(HoodIO subsystem, double degs) {
    this.subsystem = subsystem;
    this.degs = degs;

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
    subsystem.setTarget(degs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
