package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TimestampedVisionUpdate;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveSim implements SwerveDriveIO {

  final DriveTrainSimulationConfig simulationConfig;
  final SelfControlledSwerveDriveSimulation driveSimulaton;
  final Field2d field2d;

  public SwerveDriveSim() {
    simulationConfig =
        DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                COTS.ofSwerveX2(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), WHEEL_COF, 2, 11))
            .withTrackLengthTrackWidth(Inches.of(22), Inches.of(22))
            .withBumperSize(Inches.of(34), Inches.of(34))
            .withRobotMass(Pounds.of(125))
            .withCustomModuleTranslations(SwerveDriveIO.getModuleTranslations());

    driveSimulaton =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(simulationConfig, new Pose2d(1, 1, new Rotation2d())));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulaton.getDriveTrainSimulation());
    SimulatedArena.getInstance().resetFieldForAuto();
    field2d = new Field2d();
    SmartDashboard.putData("Simulation Field", field2d);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance
                    .Red, // this is correct, model all trajectories for blue side in path planner
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        activePath -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
          if (activePath.isEmpty()) return;
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(
                  activePath.get(0),
                  activePath.stream()
                      .skip(1)
                      .limit(activePath.size() - (long) 2)
                      .map(Pose2d::getTranslation)
                      .toList(),
                  activePath.get(activePath.size() - 1),
                  new TrajectoryConfig(getMaxLinearSpeedMetersPerSec(), 4.2));

          field2d.getObject("traj").setTrajectory(trajectory);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
  }

  private ChassisSpeeds getChassisSpeeds() {
    return SwerveDriveIO.kinematics.toChassisSpeeds(driveSimulaton.getMeasuredStates());
  }

  @Override
  public Transform2d getVelocity() {
    ChassisSpeeds speeds = driveSimulaton.getActualSpeedsFieldRelative();
    return new Transform2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond));
  }

  @Override
  public void periodic() {
    driveSimulaton.periodic();
    SimulatedArena.getInstance().simulationPeriodic();
    field2d.setRobotPose(getPose());
    field2d.getObject("odometry").setPose(getPose());
    Logger.recordOutput("Drive/Robot Pose", getPose());
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    driveSimulaton.runChassisSpeeds(speeds, new Translation2d(), false, true);
  }

  @Override
  public void runCharacterization(double output) {}

  @Override
  public void stop() {}

  @Override
  public void stopWithX() {}

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return null;
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return null;
  }

  @Override
  public double[] getWheelRadiusCharacterizationPositions() {
    return new double[0];
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return 0;
  }

  @Override
  public Pose2d getPose() {
    return driveSimulaton.getActualPoseInSimulationWorld();
  }

  @Override
  public void setPose(Pose2d pose) {}

  @Override
  public Rotation2d getRotation() {
    return driveSimulaton.getOdometryEstimatedPose().getRotation();
  }

  @Override
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  @Override
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    driveSimulaton.addVisionEstimation(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void addAutoVisionMeasurement(List<TimestampedVisionUpdate> timestampedVisionUpdates) {
    for (TimestampedVisionUpdate autoUpdate : timestampedVisionUpdates) {
      addVisionMeasurement(autoUpdate.pose(), autoUpdate.timestamp(), autoUpdate.stdDevs());
    }
  }
}
