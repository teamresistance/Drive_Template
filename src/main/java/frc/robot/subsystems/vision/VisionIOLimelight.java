package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIOLimelight extends VisionIO {

  /** Sets a new limelight position for the specified camera */
  void updateCameraTransform(String name, Pose3d camPose);
}
