package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.util.GeomUtil;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 4.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {
    // Deliberately empty - prevents instantiation of utility class
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      SwerveDriveIO drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      SwerveDriveIO drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(SwerveDriveIO drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  Logger.recordOutput("Drive/FF/kS", kS);
                  Logger.recordOutput("Drive/FF/kV", kV);
                  Logger.recordOutput("Drive/FF/CharacterizationComplete", true);
                }));
  }

  /**
   * Command that drives the robot to a target Transform2d using a PID controller for both the X and
   * Y positions.
   */
  public static Command goToTransform(SwerveDriveIO drive, Transform2d targetTransform) {
    // PID controllers for X and Y positions
    ProfiledPIDController pidX =
        new ProfiledPIDController(
            0.7, // KP for X (tune as needed)
            0.0, // KI for X (no integral term)
            0.0, // KD for X (no derivative term)
            new TrapezoidProfile.Constraints(
                drive.getMaxLinearSpeedMetersPerSec(),
                3.0) // Constraints on X speed (tune as needed)
            );

    ProfiledPIDController pidY =
        new ProfiledPIDController(
            0.7, // KP for Y (tune as needed)
            0.0, // KI for Y (no integral term)
            0.0, // KD for Y (no derivative term)
            new TrapezoidProfile.Constraints(
                drive.getMaxLinearSpeedMetersPerSec(),
                3.0) // Constraints on Y speed (tune as needed)
            );

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Target position (X, Y) and rotation from the target transform
    Translation2d targetTranslation = targetTransform.getTranslation();
    Rotation2d targetRotation = targetTransform.getRotation();

    return Commands.run(
            () -> {
              if (Constants.TEST_MODE) return;
              // Get current robot position
              Pose2d currentPose = drive.getPose();

              // Compute errors for X and Y
              double errorX = targetTranslation.getX() - currentPose.getX();
              double errorY = targetTranslation.getY() - currentPose.getY();
              Rotation2d errorrot = targetRotation.minus(currentPose.getRotation());

              // Use the PID controllers to calculate the required speeds to approach the target
              double speedX = pidX.calculate(currentPose.getX(), targetTranslation.getX());
              double speedY = pidY.calculate(currentPose.getY(), targetTranslation.getY());

              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetRotation.getRadians());

              // If the error is small, stop the robot (use deadband for smooth stopping)
              if (Math.abs(errorX) < 0.02
                  && Math.abs(errorY) < 0.02
                  && Math.abs(errorrot.getRadians()) < 5 * 3.14 / 180) {
                speedX = 0.0;
                speedY = 0.0;
                omega = 0.0;
              }

              // Drive the robot toward the target in field-relative coordinates
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Apply the PID-calculated speeds to the robot's drive system
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      isFlipped
                          ? -speedX * drive.getMaxLinearSpeedMetersPerSec()
                          : speedX * drive.getMaxLinearSpeedMetersPerSec(),
                      isFlipped
                          ? -speedY * drive.getMaxLinearSpeedMetersPerSec()
                          : speedY * drive.getMaxLinearSpeedMetersPerSec(),
                      omega // No angular velocity, we're just moving in XY
                      );

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              // Reset PID controllers when the command starts
              pidX.reset(drive.getPose().getX());
              pidY.reset(drive.getPose().getY());
              angleController.reset(drive.getRotation().getRadians());
            })
        .until(
            () ->
                Math.abs(targetTranslation.getX() - drive.getPose().getX()) < 0.07
                    && Math.abs(targetTranslation.getY() - drive.getPose().getY()) < 0.07
                    && Math.abs(targetRotation.minus(drive.getRotation()).getRadians())
                        < Math.toRadians(5));
  }

  public static Command goToTransformWithPathFinder(
      SwerveDriveIO drive, Transform2d targetTransform) {
    if (Constants.TEST_MODE) return new InstantCommand(() -> {});
    return AutoBuilder.pathfindToPose(
        GeomUtil.transformToPose(targetTransform),
        Constants.PATH_CONSTRAINTS,
        0.0 // Goal end velocity in meters/sec
        );
  }

  /**
   * Navigates to the first {@code Pose2d} provided, then begins following a curved path to the end
   * using the provided control points.
   *
   * @param drive The drive subsystem
   * @param start Pose to start at
   * @param end Pose to end at
   * @param controls Control points
   */
  public static Command followCurve(
      SwerveDriveIO drive, Pose2d start, Pose2d end, Pose2d... controls) {

    List<Translation2d> pts = new ArrayList<>();
    pts.add(start.getTranslation());
    for (Pose2d pose : controls) {
      pts.add(pose.getTranslation());
    }
    pts.add(end.getTranslation());

    final int degree = pts.size() - 1;
    int samples = Math.max(12, degree * 12);
    BiFunction<List<Translation2d>, Double, Translation2d> evalPoint =
        (controlPts, t) -> {
          List<Translation2d> temp = new ArrayList<>(controlPts);
          int n = temp.size();
          for (int r = 1; r < n; r++) {
            for (int i = 0; i < n - r; i++) {
              Translation2d a = temp.get(i);
              Translation2d b = temp.get(i + 1);
              double x = a.getX() * (1 - t) + b.getX() * t;
              double y = a.getY() * (1 - t) + b.getY() * t;
              temp.set(i, new Translation2d(x, y));
            }
          }
          return temp.get(0);
        };

    BiFunction<List<Translation2d>, Double, Translation2d> evalDerivative =
        (controlPts, t) -> {
          int n = controlPts.size() - 1;
          if (n <= 0) return new Translation2d(0.0, 0.0);
          List<Translation2d> diffs = new ArrayList<>();
          for (int i = 0; i < n; i++) {
            Translation2d a = controlPts.get(i);
            Translation2d b = controlPts.get(i + 1);
            diffs.add(new Translation2d((b.getX() - a.getX()) * n, (b.getY() - a.getY()) * n));
          }
          return evalPoint.apply(diffs, t);
        };

    Pose2d[] sampled = new Pose2d[samples];
    for (int i = 0; i < samples; i++) {
      double t = (double) i / (samples - 1);
      Translation2d pos = evalPoint.apply(pts, t);
      Translation2d deriv = evalDerivative.apply(pts, t);

      Rotation2d rot;
      if (Math.hypot(deriv.getX(), deriv.getY()) < 1e-6) {
        Translation2d fallback = end.getTranslation().minus(start.getTranslation());
        rot = new Rotation2d(Math.atan2(fallback.getY(), fallback.getX()));
      } else {
        rot = new Rotation2d(Math.atan2(deriv.getY(), deriv.getX()));
      }
      sampled[i] = new Pose2d(pos, rot);
    }

    return followPoses(drive, sampled);
  }

  /**
   * Navigates to the first {@code Pose2d} provided, then begins following a path constructed from
   * the provided poses.
   *
   * @param drive The drive subsystem
   * @param points {@code Pose2d} points to construct a path out of
   */
  public static Command followPoses(SwerveDriveIO drive, Pose2d... points) {
    if (points == null || points.length == 0) {
      return new InstantCommand(() -> {});
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(points);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.PATH_CONSTRAINTS,
            new IdealStartingState(
                drive.getPose().getTranslation().getDistance(points[0].getTranslation()),
                points[0].getRotation()),
            new GoalEndState(0, points[points.length - 1].getRotation()));

    path.preventFlipping = true;
    return Commands.sequence(
        AutoBuilder.pathfindToPose(
            points[0], Constants.PATH_CONSTRAINTS, Constants.PATH_CONSTRAINTS.maxVelocity()),
        AutoBuilder.followPath(path));
  }

  /**
   * Navigates to the first {@code Pose2d} provided, then begins following a path constructed from
   * the provided poses.
   *
   * @param drive The drive subsystem
   * @param transitionVelocity The speed in m/s that should be maintained from the initial pathing
   *     when starting to follow the actual path
   * @param points {@code Pose2d} points to construct a path out of
   */
  public static Command followPoses(
      SwerveDriveIO drive, double transitionVelocity, Pose2d... points) {
    if (points == null || points.length == 0) {
      return new InstantCommand(() -> {});
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(points);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.PATH_CONSTRAINTS,
            new IdealStartingState(
                drive.getPose().getTranslation().getDistance(points[0].getTranslation()),
                points[0].getRotation()),
            new GoalEndState(0, points[points.length - 1].getRotation()));

    path.preventFlipping = true;
    return Commands.sequence(
        AutoBuilder.pathfindToPose(points[0], Constants.PATH_CONSTRAINTS, transitionVelocity),
        AutoBuilder.followPath(path));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveDriveIO drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisSpeeds(0.0, 0.0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY))),
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveDriveIO.DRIVE_BASE_RADIUS) / wheelDelta;

                      Logger.recordOutput("Drive/WheelRadius/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius/GyroDelta", state.gyroDelta);
                      Logger.recordOutput("Drive/WheelRadius/Result", wheelRadius);
                      Logger.recordOutput("Drive/WheelRadius/CharacterizationComplete", true);
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
