package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDStream;
import frc.robot.subsystems.leds.LEDSubsystem;

/**
 * All this command does is add an LEDStream to the LEDSubsystem. It also runs the stream for a
 * specified amount of seconds, which is necessary for streams that are meant to be temporary.
 * However, if the stream is meant to be indefinite, no inut is needed for seconds, and the stream
 * will run until it is no longer active.
 */
public class LEDCommand extends Command {
  private final LEDSubsystem leds;
  private final LEDStream ledStream;
  private final double seconds;

  // With seconds
  public LEDCommand(LEDSubsystem leds, LEDStream ledStream, double seconds) {
    this.leds = leds;
    this.ledStream = ledStream;
    this.seconds = seconds;
    addRequirements(leds);
  }

  // Without seconds
  public LEDCommand(LEDSubsystem leds, LEDStream ledStream) {
    this(leds, ledStream, -1);
  }

  @Override
  public void execute() {

    // With seconds
    if (seconds != -1) {
      ledStream.runForSeconds(seconds);

      // Without seconds
    } else {
      ledStream.run();
    }
    leds.addStream(ledStream);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
