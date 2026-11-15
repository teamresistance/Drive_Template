package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ContinuousLEDCommand;
import frc.robot.commands.ContinuousTurretCommand;
import frc.robot.commands.ContinuousVisionStdDevCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodSetTargetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodReal;
import frc.robot.subsystems.shooter.hood.HoodSim;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretReal;
import frc.robot.subsystems.shooter.turret.TurretSim;
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
  private VisionIOPhoton visionPhoton;
  private VisionIOLimelight visionLimelight;
  private final LEDSubsystem leds;
  // private final IntakeIO intake;
  private final HoodIO hood;
  private final TurretIO turret;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

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
        // intake = new IntakeReal();
        hood = new HoodReal();
        turret = new TurretReal();
        break;

      case SIM: // SIMULATED robot, instantiate sim implementations
        drive =
            new SwerveDriveReal(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // intake = new IntakeSim();
        hood = new HoodSim();
        turret = new TurretSim();
        break;

      default: // REPLAYED robot, instantiate blank implementations or default ones
        drive =
            new SwerveDriveReal(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // intake = new IntakeReal();
        hood = new HoodReal();
        turret = new TurretReal();
    }

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

  /** All triggers and ' button bindings for driver control go here. */
  private void configureButtonBindings() {
    // Normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Continuous turret command
    turret.setDefaultCommand(new ContinuousTurretCommand(turret, drive));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // When the left trigger is held, the intake deploys
    // driver.leftTrigger().onTrue(new IntakeDeployCommand(intake));

    // When the left trigger is released, the intake goes into its "kinda in" position
    // driver.leftTrigger().onFalse(new IntakeKindaInCommand(intake));

    // When the left bumper is pressed, the intake goes into its fully in position
    // driver.leftBumper().onTrue(new IntakeFullyInCommand(intake));

    // When the right stick is pressed, the hood angle goes to 13 degrees
    driver
        .rightStick()
        .onTrue(new HoodSetTargetCommand(hood, Constants.SHOOTER_HOOD_MOTOR_START_DEG));

    // When A is pressed, the hood angle goes to 20 degrees
    driver.a().onTrue(new HoodSetTargetCommand(hood, 20));

    // When X is pressed, the hood angle goes to 30 degrees
    driver.x().onTrue(new HoodSetTargetCommand(hood, 30));

    // When B is pressed, the hood angle goes to 40 degrees
    driver.b().onTrue(new HoodSetTargetCommand(hood, 40));

    // When Y is pressed, the hood angle goes to 45 degrees
    driver.x().onTrue(new HoodSetTargetCommand(hood, Constants.SHOOTER_HOOD_MOTOR_MAX_DEG));
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
