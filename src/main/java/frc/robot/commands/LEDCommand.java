package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDStream;
import frc.robot.subsystems.leds.LEDSubsystem;

/**
 * All this command does is add an LEDStream to the LEDSubsystem. It also runs the stream for a
 * specified amount of seconds, which is necessary for streams that are meant to be temporary.
 */
public class LEDCommand extends Command {
  private final LEDSubsystem leds;
  private final LEDStream ledStream;
  private final double seconds;

  public LEDCommand(LEDSubsystem leds, LEDStream ledStream, double seconds) {
    this.leds = leds;
    this.ledStream = ledStream;
    this.seconds = seconds;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    ledStream.runForSeconds(seconds);
    leds.addStream(ledStream);
  }
}
