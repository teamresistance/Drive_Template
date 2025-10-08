package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("GeomUtil Tests")
class GeomUtilTest {

  private static final double DELTA = 1e-9;

  @Test
  @DisplayName("Constructor throws IllegalStateException")
  void testConstructorThrows() throws Exception {
    java.lang.reflect.Constructor<GeomUtil> constructor = GeomUtil.class.getDeclaredConstructor();
    constructor.setAccessible(true);
    assertThrows(
        java.lang.reflect.InvocationTargetException.class, () -> constructor.newInstance());
  }

  @Test
  @DisplayName("translationToTransform with Translation2d creates correct transform")
  void testTranslationToTransformWithTranslation2d() {
    Translation2d translation = new Translation2d(3.0, 4.0);
    Transform2d transform = GeomUtil.translationToTransform(translation);

    assertEquals(3.0, transform.getX(), DELTA);
    assertEquals(4.0, transform.getY(), DELTA);
    assertEquals(0.0, transform.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("translationToTransform with x,y creates correct transform")
  void testTranslationToTransformWithXY() {
    Transform2d transform = GeomUtil.translationToTransform(5.0, -2.0);

    assertEquals(5.0, transform.getX(), DELTA);
    assertEquals(-2.0, transform.getY(), DELTA);
    assertEquals(0.0, transform.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("rotationToTransform creates correct transform")
  void testRotationToTransform() {
    Rotation2d rotation = Rotation2d.fromDegrees(90.0);
    Transform2d transform = GeomUtil.rotationToTransform(rotation);

    assertEquals(0.0, transform.getX(), DELTA);
    assertEquals(0.0, transform.getY(), DELTA);
    assertEquals(Math.PI / 2, transform.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("poseToTransform converts correctly")
  void testPoseToTransform() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0));
    Transform2d transform = GeomUtil.poseToTransform(pose);

    assertEquals(1.0, transform.getX(), DELTA);
    assertEquals(2.0, transform.getY(), DELTA);
    assertEquals(Math.PI / 4, transform.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("transformToPose converts correctly")
  void testTransformToPose() {
    Transform2d transform = new Transform2d(3.0, 4.0, Rotation2d.fromDegrees(180.0));
    Pose2d pose = GeomUtil.transformToPose(transform);

    assertEquals(3.0, pose.getX(), DELTA);
    assertEquals(4.0, pose.getY(), DELTA);
    assertEquals(Math.PI, pose.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("translationToPose creates correct pose")
  void testTranslationToPose() {
    Translation2d translation = new Translation2d(6.0, 7.0);
    Pose2d pose = GeomUtil.translationToPose(translation);

    assertEquals(6.0, pose.getX(), DELTA);
    assertEquals(7.0, pose.getY(), DELTA);
    assertEquals(0.0, pose.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("rotationToPose creates correct pose")
  void testRotationToPose() {
    Rotation2d rotation = Rotation2d.fromDegrees(270.0);
    Pose2d pose = GeomUtil.rotationToPose(rotation);

    assertEquals(0.0, pose.getX(), DELTA);
    assertEquals(0.0, pose.getY(), DELTA);
    // 270 degrees = -90 degrees in normalized form
    assertEquals(rotation.getRadians(), pose.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("multiplyTwist scales all components correctly")
  void testMultiplyTwist() {
    Twist2d twist = new Twist2d(1.0, 2.0, 0.5);
    Twist2d scaled = GeomUtil.multiplyTwist(twist, 3.0);

    assertEquals(3.0, scaled.dx, DELTA);
    assertEquals(6.0, scaled.dy, DELTA);
    assertEquals(1.5, scaled.dtheta, DELTA);
  }

  @Test
  @DisplayName("multiplyTwist with zero factor returns zero twist")
  void testMultiplyTwistZero() {
    Twist2d twist = new Twist2d(1.0, 2.0, 0.5);
    Twist2d scaled = GeomUtil.multiplyTwist(twist, 0.0);

    assertEquals(0.0, scaled.dx, DELTA);
    assertEquals(0.0, scaled.dy, DELTA);
    assertEquals(0.0, scaled.dtheta, DELTA);
  }

  @Test
  @DisplayName("pose3dToTransform3d converts correctly")
  void testPose3dToTransform3d() {
    Pose3d pose = new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3));
    Transform3d transform = GeomUtil.pose3dToTransform3d(pose);

    assertEquals(1.0, transform.getX(), DELTA);
    assertEquals(2.0, transform.getY(), DELTA);
    assertEquals(3.0, transform.getZ(), DELTA);
    assertEquals(pose.getRotation().getX(), transform.getRotation().getX(), DELTA);
    assertEquals(pose.getRotation().getY(), transform.getRotation().getY(), DELTA);
    assertEquals(pose.getRotation().getZ(), transform.getRotation().getZ(), DELTA);
  }

  @Test
  @DisplayName("transform3dToPose3d converts correctly")
  void testTransform3dToPose3d() {
    Transform3d transform = new Transform3d(4.0, 5.0, 6.0, new Rotation3d(0.4, 0.5, 0.6));
    Pose3d pose = GeomUtil.transform3dToPose3d(transform);

    assertEquals(4.0, pose.getX(), DELTA);
    assertEquals(5.0, pose.getY(), DELTA);
    assertEquals(6.0, pose.getZ(), DELTA);
    assertEquals(transform.getRotation().getX(), pose.getRotation().getX(), DELTA);
    assertEquals(transform.getRotation().getY(), pose.getRotation().getY(), DELTA);
    assertEquals(transform.getRotation().getZ(), pose.getRotation().getZ(), DELTA);
  }

  @Test
  @DisplayName("translation3dTo2dXY extracts X and Y correctly")
  void testTranslation3dTo2dXY() {
    Translation3d translation3d = new Translation3d(1.0, 2.0, 3.0);
    Translation2d translation2d = GeomUtil.translation3dTo2dXY(translation3d);

    assertEquals(1.0, translation2d.getX(), DELTA);
    assertEquals(2.0, translation2d.getY(), DELTA);
  }

  @Test
  @DisplayName("translation3dTo2dXZ extracts X and Z correctly")
  void testTranslation3dTo2dXZ() {
    Translation3d translation3d = new Translation3d(1.0, 2.0, 3.0);
    Translation2d translation2d = GeomUtil.translation3dTo2dXZ(translation3d);

    assertEquals(1.0, translation2d.getX(), DELTA);
    assertEquals(3.0, translation2d.getY(), DELTA);
  }

  @Test
  @DisplayName("Round-trip pose to transform to pose preserves values")
  void testPoseTransformRoundTrip() {
    Pose2d original = new Pose2d(10.0, 20.0, Rotation2d.fromDegrees(60.0));
    Transform2d transform = GeomUtil.poseToTransform(original);
    Pose2d result = GeomUtil.transformToPose(transform);

    assertEquals(original.getX(), result.getX(), DELTA);
    assertEquals(original.getY(), result.getY(), DELTA);
    assertEquals(original.getRotation().getRadians(), result.getRotation().getRadians(), DELTA);
  }

  @Test
  @DisplayName("Round-trip 3d pose to transform to pose preserves values")
  void testPose3dTransform3dRoundTrip() {
    Pose3d original = new Pose3d(10.0, 20.0, 30.0, new Rotation3d(0.1, 0.2, 0.3));
    Transform3d transform = GeomUtil.pose3dToTransform3d(original);
    Pose3d result = GeomUtil.transform3dToPose3d(transform);

    assertEquals(original.getX(), result.getX(), DELTA);
    assertEquals(original.getY(), result.getY(), DELTA);
    assertEquals(original.getZ(), result.getZ(), DELTA);
    assertEquals(original.getRotation().getX(), result.getRotation().getX(), DELTA);
    assertEquals(original.getRotation().getY(), result.getRotation().getY(), DELTA);
    assertEquals(original.getRotation().getZ(), result.getRotation().getZ(), DELTA);
  }
}
