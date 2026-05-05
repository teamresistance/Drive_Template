package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ContinuousLEDCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // photon vision cameras
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back-left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back-right");
  private final Alert cameraFailureAlert;

  // Subsystems
  private final SwerveDriveIO drive;
  private VisionIOPhoton vision;
  private final LEDSubsystem leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    // instantiate any special subsystems that do not get differing implementations here
    leds = new LEDSubsystem();

    switch (Constants.CURRENT_MODE) {
      case REAL: // REAL robot, instantiate real implementations
        drive =
            new SwerveDriveReal(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM: // SIMULATED robot, instantiate sim implementations
        drive = new SwerveDriveSim();
        break;

      default: // REPLAYED robot, instantiate blank implementations or default ones
        drive =
            new SwerveDriveReal(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
    }

    vision = configureAprilTagVision();
    configureNamedCommands();
    autoChooser = configureAutos();
    configureButtonBindings();
    configureFeedback();
    cameraFailureAlert = new Alert("Camera system failure", Alert.AlertType.kError);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Stop", Commands.runOnce(drive::stop, drive));
  }

  private LoggedDashboardChooser<Command> configureAutos() {
    // Set up pathplanner auto routines
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    chooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    chooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    chooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    return chooser;
  }

  /** Returns the AprilTag vision system with PhotonVision cameras. */
  private VisionIOPhoton configureAprilTagVision() {
    try {
      vision =
          switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY ->
                new VisionRealPhoton(
                    frontLeftCamera, frontRightCamera, backRightCamera, backLeftCamera);
            case SIM ->
                new VisionSimPhoton(
                    frontLeftCamera, frontRightCamera, backRightCamera, backLeftCamera);
          };
      vision.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);

    } catch (IOException e) {
      if (cameraFailureAlert != null) {
        cameraFailureAlert.set(true);
      }

      Logger.recordOutput("Vision/FieldLayoutLoadError", e.getMessage());
      return null; // Return null on failure for proper error handling
    }
    return vision;
  }

  /** All triggers and button bindings for driver control go here. */
  private void configureButtonBindings() {
    // Normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  /** Any triggers for driver feedback (i.e. rumbles, the LED command, etc.) should go here. */
  private void configureFeedback() {
    leds.setDefaultCommand(new ContinuousLEDCommand(leds, drive));
  }

  /** This method should return the Command to run during auto scheduled by Robot.java */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.get();
    if (autoCommand == null) {
      Logger.recordOutput("Auto/NoCommandSelected", true);
      return Commands.none(); // Return empty command if no auto selected
    }

    return autoCommand;
  }
}
