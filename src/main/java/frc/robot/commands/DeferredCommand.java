package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** Evaluates a command's parameters later. */
public class DeferredCommand extends Command {
  private final Supplier<Command> factory;
  private Command delegate;

  /**
   * Creates a Deferred Command that will initialize its child command every time it runs.
   *
   * @param factory supplier that constructs the delegate command at runtime
   * @param requirements optional subsystem requirements to add to this wrapper
   */
  public DeferredCommand(Supplier<Command> factory, Subsystem... requirements) {
    this.factory = factory;
    if (requirements != null && requirements.length > 0) {
      addRequirements(requirements);
    }
  }

  @Override
  public void initialize() {
    delegate = factory.get();
    if (delegate != null) {
      delegate.initialize();
    }
  }

  @Override
  public void execute() {
    if (delegate != null) {
      delegate.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return delegate == null || delegate.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (delegate != null) {
      delegate.end(interrupted);
    }
    delegate = null;
  }

  @Override
  public boolean runsWhenDisabled() {
    return delegate != null && delegate.runsWhenDisabled();
  }
}
