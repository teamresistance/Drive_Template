package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
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
  final SelfControlledSwerveDriveSimulation driveSimulation;
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

    driveSimulation =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(simulationConfig, new Pose2d(1, 1, new Rotation2d())));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    SimulatedArena.getInstance().resetFieldForAuto();
    field2d = new Field2d();
    SmartDashboard.putData("Simulation Field", field2d);

    configure(); // Configure AutoBuilder and PathPlanner
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return SwerveDriveIO.kinematics.toChassisSpeeds(driveSimulation.getMeasuredStates());
  }

  @Override
  public Transform2d getVelocity() {
    ChassisSpeeds speeds = driveSimulation.getActualSpeedsFieldRelative();
    return new Transform2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond));
  }

  @Override
  public void periodic() {
    driveSimulation.periodic();
    SimulatedArena.getInstance().simulationPeriodic();
    field2d.setRobotPose(getPose());
    field2d.getObject("odometry").setPose(getPose());
    Logger.recordOutput("Drive/Robot Pose", getPose());
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, true);
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
    return driveSimulation.getActualPoseInSimulationWorld();
  }

  @Override
  public void setPose(Pose2d pose) {}

  @Override
  public Rotation2d getRotation() {
    return driveSimulation.getOdometryEstimatedPose().getRotation();
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
    driveSimulation.addVisionEstimation(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void addAutoVisionMeasurement(List<TimestampedVisionUpdate> timestampedVisionUpdates) {
    for (TimestampedVisionUpdate autoUpdate : timestampedVisionUpdates) {
      addVisionMeasurement(autoUpdate.pose(), autoUpdate.timestamp(), autoUpdate.stdDevs());
    }
  }
}
