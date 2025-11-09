package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.TimestampedVisionUpdate;
import java.util.List;

public interface SwerveDrive extends Subsystem {

  Transform2d getVelocity();

  void periodic();

  void runVelocity(ChassisSpeeds speeds);

  void runCharacterization(double output);

  void stop();

  void stopWithX();

  Command sysIdQuasistatic(SysIdRoutine.Direction direction);

  Command sysIdDynamic(SysIdRoutine.Direction direction);

  double[] getWheelRadiusCharacterizationPositions();

  double getFFCharacterizationVelocity();

  Pose2d getPose();

  void setPose(Pose2d pose);

  Rotation2d getRotation();

  double getMaxLinearSpeedMetersPerSec();

  double getMaxAngularSpeedRadPerSec();

  void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  void addAutoVisionMeasurement(List<TimestampedVisionUpdate> timestampedVisionUpdates);
}
