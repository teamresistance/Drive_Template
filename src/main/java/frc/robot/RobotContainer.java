package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ContinuousLEDCommand;
import frc.robot.commands.ContinuousVisionStdDevCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import java.io.IOException;
import java.nio.file.Files;
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
  private VisionIOPhoton visionPhoton;
  private VisionIOLimelight visionLimelight;
  private final LEDSubsystem leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // BLine path following
  private final FollowPath.Builder blineBuilder;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    // instantiate any special subsystems that do not get differing implementations here
    leds = new LEDSubsystem();
    visionLimelight = new VisionRealLimelight("limelight");

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
        drive =
            new SwerveDriveReal(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
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

    blineBuilder = drive.getBLineBuilder();

    visionPhoton = configureAprilTagVision();
    visionLimelight.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);
    configureNamedCommands();
    autoChooser = configureAutos();
    configureButtonBindings();
    configureFeedback();
    cameraFailureAlert = new Alert("Camera system failure", Alert.AlertType.kError);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Stop", Commands.runOnce(drive::stop, drive));
    FollowPath.registerEventTrigger("Stop", () -> drive.stop());
  }

  private LoggedDashboardChooser<Command> configureAutos() {
    // Set up pathplanner auto routines
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    java.nio.file.Path blinePaths =
        Filesystem.getDeployDirectory().toPath().resolve("autos").resolve("paths");
    if (Files.isDirectory(blinePaths)) {
      try (var files = Files.list(blinePaths)) {
        files
            .filter(file -> file.toString().endsWith(".json"))
            .sorted()
            .forEach(file -> {
              String name = file.getFileName().toString();
              String stem = name.substring(0, name.length() - 5);
              try {
                Path path = new Path(stem);
                if (!path.isValid()) {
                  throw new IllegalArgumentException("BLine path is invalid: " + stem);
                }
                Command blineCommand = blineBuilder.withPoseReset(drive::setPose).build(path);
                blineBuilder.withPoseReset(ignored -> {});
                chooser.addOption("BLine - " + stem, blineCommand);
              } catch (RuntimeException e) {
                Logger.recordOutput("BLine/LoadError/" + stem, e.getMessage());
                DriverStation.reportError(
                    "Unable to load BLine path " + stem + ": " + e.getMessage(), false);
              }
            });
      } catch (IOException e) {
        Logger.recordOutput("BLine/LoadError", e.getMessage());
        DriverStation.reportError("Unable to list BLine paths: " + e.getMessage(), false);
      }
    }

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

  /** Returns the AprilTag visionPhoton system with PhotonVision cameras. */
  private VisionIOPhoton configureAprilTagVision() {
    try {
      visionPhoton =
          switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY ->
                new VisionRealPhoton(
                    frontLeftCamera, frontRightCamera, backRightCamera, backLeftCamera);
            case SIM ->
                new VisionSimPhoton(
                    frontLeftCamera, frontRightCamera, backRightCamera, backLeftCamera);
          };
      visionPhoton.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);

    } catch (IOException e) {
      if (cameraFailureAlert != null) {
        cameraFailureAlert.set(true);
      }

      Logger.recordOutput("Vision/FieldLayoutLoadError", e.getMessage());
      return null; // Return null on failure for proper error handling
    }
    return visionPhoton;
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

    // this can be either vision or limelight, it just needs to be something's default
    visionPhoton.setDefaultCommand(
        new ContinuousVisionStdDevCommand(drive, visionPhoton, visionLimelight));
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
