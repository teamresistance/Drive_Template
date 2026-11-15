package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeKindaInCommand extends Command {

  // Declares the intake subsystem
  IntakeIO subsystem;

  // Creates a new IntakeKindaInCommand.
  public IntakeKindaInCommand(IntakeIO subsystem) {

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
    subsystem.kindaIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Stops the command if the slapdown motor is within 0.25 degrees of 20 degrees
    return subsystem.withinTolerance(Constants.SLAPDOWN_KINDA_IN);
  }
}
