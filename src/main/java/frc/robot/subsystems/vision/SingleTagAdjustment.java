package frc.robot.subsystems.vision;

import frc.robot.util.LoggedTunableNumber;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Utility class for managing adjustment factors for single AprilTag vision measurements. Provides
 * dynamic tuning of vision measurement confidence based on tag ID.
 */
public class SingleTagAdjustment {

  private SingleTagAdjustment() {
    throw new IllegalStateException("Utility class");
  }

  // Default adjustment factors for each tag ID.  These are applied if no dynamic override exists.
  private static final double[] DEFAULT_TAG_ADJUSTMENTS = {
    1.0, // 1
    1.0, // 2
    1.0, // 3
    1000.0, // 4
    1000.0, // 5
    0.8, // 6
    0.8, // 7
    0.8, // 8
    0.8, // 9
    0.8, // 10
    0.8, // 11
    1.0, // 12
    1.0, // 13
    1000.0, // 14
    1000.0, // 15
    1.0, // 16
    0.8, // 17
    0.8, // 18
    0.8, // 19
    0.8, // 20
    0.8, // 21
    0.8, // 22
  };

  // A map to store dynamic adjustments (overrides) for specific tag IDs.  This allows runtime
  // tuning.
  private static final Map<Integer, Double> dynamicAdjustments = new ConcurrentHashMap<>();

  // Tunable numbers for each tag's adjustment.  These provide a way to adjust values through a
  // dashboard.
  private static final LoggedTunableNumber tag1Adjustment =
      new LoggedTunableNumber("Vision/Tag1Adjustment", DEFAULT_TAG_ADJUSTMENTS[0]);
  private static final LoggedTunableNumber tag2Adjustment =
      new LoggedTunableNumber("Vision/Tag2Adjustment", DEFAULT_TAG_ADJUSTMENTS[1]);
  private static final LoggedTunableNumber tag3Adjustment =
      new LoggedTunableNumber("Vision/Tag3Adjustment", DEFAULT_TAG_ADJUSTMENTS[2]);
  private static final LoggedTunableNumber tag4Adjustment =
      new LoggedTunableNumber("Vision/Tag4Adjustment", DEFAULT_TAG_ADJUSTMENTS[3]);
  private static final LoggedTunableNumber tag5Adjustment =
      new LoggedTunableNumber("Vision/Tag5Adjustment", DEFAULT_TAG_ADJUSTMENTS[4]);
  private static final LoggedTunableNumber tag6Adjustment =
      new LoggedTunableNumber("Vision/Tag6Adjustment", DEFAULT_TAG_ADJUSTMENTS[5]);
  private static final LoggedTunableNumber tag7Adjustment =
      new LoggedTunableNumber("Vision/Tag7Adjustment", DEFAULT_TAG_ADJUSTMENTS[6]);
  private static final LoggedTunableNumber tag8Adjustment =
      new LoggedTunableNumber("Vision/Tag8Adjustment", DEFAULT_TAG_ADJUSTMENTS[7]);
  private static final LoggedTunableNumber tag9Adjustment =
      new LoggedTunableNumber("Vision/Tag9Adjustment", DEFAULT_TAG_ADJUSTMENTS[8]);
  private static final LoggedTunableNumber tag10Adjustment =
      new LoggedTunableNumber("Vision/Tag10Adjustment", DEFAULT_TAG_ADJUSTMENTS[9]);
  private static final LoggedTunableNumber tag11Adjustment =
      new LoggedTunableNumber("Vision/Tag11Adjustment", DEFAULT_TAG_ADJUSTMENTS[10]);
  private static final LoggedTunableNumber tag12Adjustment =
      new LoggedTunableNumber("Vision/Tag12Adjustment", DEFAULT_TAG_ADJUSTMENTS[11]);
  private static final LoggedTunableNumber tag13Adjustment =
      new LoggedTunableNumber("Vision/Tag13Adjustment", DEFAULT_TAG_ADJUSTMENTS[12]);
  private static final LoggedTunableNumber tag14Adjustment =
      new LoggedTunableNumber("Vision/Tag14Adjustment", DEFAULT_TAG_ADJUSTMENTS[13]);
  private static final LoggedTunableNumber tag15Adjustment =
      new LoggedTunableNumber("Vision/Tag15Adjustment", DEFAULT_TAG_ADJUSTMENTS[14]);
  private static final LoggedTunableNumber tag16Adjustment =
      new LoggedTunableNumber("Vision/Tag16Adjustment", DEFAULT_TAG_ADJUSTMENTS[15]);

  // Array to hold all the tunable adjustment numbers.  This simplifies updating them.
  private static final LoggedTunableNumber[] tagAdjustments = {
    tag1Adjustment,
    tag2Adjustment,
    tag3Adjustment,
    tag4Adjustment,
    tag5Adjustment,
    tag6Adjustment,
    tag7Adjustment,
    tag8Adjustment,
    tag9Adjustment,
    tag10Adjustment,
    tag11Adjustment,
    tag12Adjustment,
    tag13Adjustment,
    tag14Adjustment,
    tag15Adjustment,
    tag16Adjustment
  };

  /**
   * Retrieves the adjustment factor for a given AprilTag ID.
   *
   * @param tagId The ID of the AprilTag.
   * @return The adjustment factor. Defaults to 1.0 if no adjustment is found. First checks for a
   *     dynamic adjustment, then falls back to the default adjustments. Returns 1.0 if tagId is out
   *     of bounds.
   */
  public static double getAdjustmentForTag(int tagId) {
    return dynamicAdjustments.getOrDefault(
        tagId,
        (tagId >= 1 && tagId <= DEFAULT_TAG_ADJUSTMENTS.length)
            ? DEFAULT_TAG_ADJUSTMENTS[tagId - 1]
            : 1.0);
  }

  /**
   * Sets a dynamic adjustment factor for a specific AprilTag ID. This overrides the default
   * adjustment.
   *
   * @param tagId The ID of the AprilTag.
   * @param adjustment The new adjustment factor.
   */
  public static void setAdjustmentForTag(int tagId, double adjustment) {
    dynamicAdjustments.put(tagId, adjustment);
  }

  /**
   * Updates the dynamic tag adjustments from the logged tunable numbers. This method should be
   * called periodically (e.g., in a periodic function) to apply any changes made through the
   * dashboard.
   */
  public static void updateLoggedTagAdjustments() {
    for (int i = 0; i < tagAdjustments.length; i++) {
      // Check if the tunable number has changed since the last check.
      if (tagAdjustments[i].hasChanged(tagAdjustments[i].hashCode())) {
        // Tag IDs are 1-indexed, array is 0-indexed.
        setAdjustmentForTag(i + 1, tagAdjustments[i].get());
      }
    }
  }
}
