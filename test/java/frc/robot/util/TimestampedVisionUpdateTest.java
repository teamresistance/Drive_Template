package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("TimestampedVisionUpdate Tests")
class TimestampedVisionUpdateTest {

  private static final double DELTA = 1e-9;

  @Test
  @DisplayName("Constructor creates record with all fields")
  void testConstructor() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0));
    double timestamp = 123.456;
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, 0.1);
    stdDevs.set(1, 0, 0.2);
    stdDevs.set(2, 0, 0.3);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, timestamp, stdDevs);

    assertNotNull(update);
    assertEquals(pose, update.pose());
    assertEquals(timestamp, update.timestamp(), DELTA);
    assertEquals(stdDevs, update.stdDevs());
  }

  @Test
  @DisplayName("Pose accessor returns correct pose")
  void testPoseAccessor() {
    Pose2d pose = new Pose2d(5.0, 10.0, Rotation2d.fromDegrees(90.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 0.0, stdDevs);

    Pose2d retrievedPose = update.pose();
    assertEquals(5.0, retrievedPose.getX(), DELTA);
    assertEquals(10.0, retrievedPose.getY(), DELTA);
    assertEquals(Math.PI / 2, retrievedPose.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("Timestamp accessor returns correct timestamp")
  void testTimestampAccessor() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    double timestamp = 987.654321;

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, timestamp, stdDevs);

    assertEquals(timestamp, update.timestamp(), DELTA);
  }

  @Test
  @DisplayName("StdDevs accessor returns correct standard deviations")
  void testStdDevsAccessor() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, 0.5);
    stdDevs.set(1, 0, 1.0);
    stdDevs.set(2, 0, 1.5);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 0.0, stdDevs);

    Matrix<N3, N1> retrievedStdDevs = update.stdDevs();
    assertEquals(0.5, retrievedStdDevs.get(0, 0), DELTA);
    assertEquals(1.0, retrievedStdDevs.get(1, 0), DELTA);
    assertEquals(1.5, retrievedStdDevs.get(2, 0), DELTA);
  }

  @Test
  @DisplayName("Record equality works correctly")
  void testEquality() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, 0.1);
    stdDevs.set(1, 0, 0.2);
    stdDevs.set(2, 0, 0.3);

    TimestampedVisionUpdate update1 = new TimestampedVisionUpdate(pose, 100.0, stdDevs);
    TimestampedVisionUpdate update2 = new TimestampedVisionUpdate(pose, 100.0, stdDevs);

    assertEquals(update1, update2);
  }

  @Test
  @DisplayName("Record inequality for different poses")
  void testInequalityDifferentPose() {
    Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update1 = new TimestampedVisionUpdate(pose1, 100.0, stdDevs);
    TimestampedVisionUpdate update2 = new TimestampedVisionUpdate(pose2, 100.0, stdDevs);

    assertNotEquals(update1, update2);
  }

  @Test
  @DisplayName("Record inequality for different timestamps")
  void testInequalityDifferentTimestamp() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update1 = new TimestampedVisionUpdate(pose, 100.0, stdDevs);
    TimestampedVisionUpdate update2 = new TimestampedVisionUpdate(pose, 200.0, stdDevs);

    assertNotEquals(update1, update2);
  }

  @Test
  @DisplayName("Record with zero timestamp")
  void testZeroTimestamp() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 0.0, stdDevs);

    assertEquals(0.0, update.timestamp(), DELTA);
  }

  @Test
  @DisplayName("Record with negative timestamp")
  void testNegativeTimestamp() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, -10.0, stdDevs);

    assertEquals(-10.0, update.timestamp(), DELTA);
  }

  @Test
  @DisplayName("Record with origin pose")
  void testOriginPose() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 50.0, stdDevs);

    assertEquals(0.0, update.pose().getX(), DELTA);
    assertEquals(0.0, update.pose().getY(), DELTA);
    assertEquals(0.0, update.pose().getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("Record with all zero standard deviations")
  void testZeroStdDevs() {
    Pose2d pose = new Pose2d();
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    // All zeros by default

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 0.0, stdDevs);

    assertEquals(0.0, update.stdDevs().get(0, 0), DELTA);
    assertEquals(0.0, update.stdDevs().get(1, 0), DELTA);
    assertEquals(0.0, update.stdDevs().get(2, 0), DELTA);
  }

  @Test
  @DisplayName("Record hashCode consistency")
  void testHashCodeConsistency() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);

    TimestampedVisionUpdate update1 = new TimestampedVisionUpdate(pose, 100.0, stdDevs);
    TimestampedVisionUpdate update2 = new TimestampedVisionUpdate(pose, 100.0, stdDevs);

    assertEquals(update1.hashCode(), update2.hashCode());
  }

  @Test
  @DisplayName("Record toString contains field information")
  void testToString() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, 0.1);

    TimestampedVisionUpdate update = new TimestampedVisionUpdate(pose, 100.0, stdDevs);

    String str = update.toString();
    assertNotNull(str);
    assertTrue(str.contains("TimestampedVisionUpdate"));
  }

  @Test
  @DisplayName("Multiple instances are independent")
  void testIndependence() {
    Pose2d pose1 = new Pose2d(1.0, 1.0, new Rotation2d());
    Pose2d pose2 = new Pose2d(2.0, 2.0, new Rotation2d());
    Matrix<N3, N1> stdDevs1 = new Matrix<>(N3.instance, N1.instance);
    Matrix<N3, N1> stdDevs2 = new Matrix<>(N3.instance, N1.instance);
    stdDevs1.set(0, 0, 0.5);
    stdDevs2.set(0, 0, 1.0);

    TimestampedVisionUpdate update1 = new TimestampedVisionUpdate(pose1, 10.0, stdDevs1);
    TimestampedVisionUpdate update2 = new TimestampedVisionUpdate(pose2, 20.0, stdDevs2);

    // Verify they don't interfere with each other
    assertEquals(1.0, update1.pose().getX(), DELTA);
    assertEquals(2.0, update2.pose().getX(), DELTA);
    assertEquals(10.0, update1.timestamp(), DELTA);
    assertEquals(20.0, update2.timestamp(), DELTA);
    assertEquals(0.5, update1.stdDevs().get(0, 0), DELTA);
    assertEquals(1.0, update2.stdDevs().get(0, 0), DELTA);
  }
}
