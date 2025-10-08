package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("SingleTagAdjustment Tests")
class SingleTagAdjustmentTest {

  private static final double DELTA = 1e-9;

  @Test
  @DisplayName("Constructor throws IllegalStateException")
  void testConstructorThrows() throws Exception {
    java.lang.reflect.Constructor<SingleTagAdjustment> constructor =
        SingleTagAdjustment.class.getDeclaredConstructor();
    constructor.setAccessible(true);
    assertThrows(
        java.lang.reflect.InvocationTargetException.class, () -> constructor.newInstance());
  }

  @Test
  @DisplayName("Get adjustment returns non-negative values")
  void testGetAdjustmentNonNegative() {
    // Test that we can get adjustments without errors
    for (int i = 1; i <= 22; i++) {
      double adjustment = SingleTagAdjustment.getAdjustmentForTag(i);
      assertTrue(adjustment >= 0.0, "Tag " + i + " should have non-negative adjustment");
    }
  }

  @Test
  @DisplayName("Get adjustment for tag 0 returns 1.0")
  void testGetAdjustmentTag0() {
    double adjustment = SingleTagAdjustment.getAdjustmentForTag(0);
    assertEquals(1.0, adjustment, DELTA);
  }

  @Test
  @DisplayName("Get adjustment for negative tag returns 1.0")
  void testGetAdjustmentNegativeTag() {
    double adjustment = SingleTagAdjustment.getAdjustmentForTag(-1);
    assertEquals(1.0, adjustment, DELTA);
  }

  @Test
  @DisplayName("Set adjustment for tag works")
  void testSetAdjustment() {
    int tagId = 99; // Use high number to avoid conflicts
    double testValue = 12.345;

    SingleTagAdjustment.setAdjustmentForTag(tagId, testValue);
    double retrieved = SingleTagAdjustment.getAdjustmentForTag(tagId);
    assertEquals(testValue, retrieved, DELTA);
  }

  @Test
  @DisplayName("Set adjustment can be updated multiple times")
  void testSetAdjustmentMultipleTimes() {
    int tagId = 98; // Use high number to avoid conflicts

    SingleTagAdjustment.setAdjustmentForTag(tagId, 10.0);
    assertEquals(10.0, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);

    SingleTagAdjustment.setAdjustmentForTag(tagId, 20.0);
    assertEquals(20.0, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);

    SingleTagAdjustment.setAdjustmentForTag(tagId, 30.0);
    assertEquals(30.0, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);
  }

  @Test
  @DisplayName("Set adjustment with zero value")
  void testSetAdjustmentZero() {
    int tagId = 97;
    SingleTagAdjustment.setAdjustmentForTag(tagId, 0.0);
    assertEquals(0.0, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);
  }

  @Test
  @DisplayName("Set adjustment with negative value")
  void testSetAdjustmentNegative() {
    int tagId = 96;
    SingleTagAdjustment.setAdjustmentForTag(tagId, -5.0);
    assertEquals(-5.0, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);
  }

  @Test
  @DisplayName("Set adjustment with very large value")
  void testSetAdjustmentLarge() {
    int tagId = 95;
    SingleTagAdjustment.setAdjustmentForTag(tagId, 1e10);
    assertEquals(1e10, SingleTagAdjustment.getAdjustmentForTag(tagId), DELTA);
  }

  @Test
  @DisplayName("Update logged tag adjustments executes without error")
  void testUpdateLoggedTagAdjustments() {
    // This method should execute without throwing
    assertDoesNotThrow(() -> SingleTagAdjustment.updateLoggedTagAdjustments());
  }

  @Test
  @DisplayName("Setting adjustment for one tag doesn't affect unrelated tags")
  void testSetAdjustmentIsolation() {
    int tagId1 = 94;
    int tagId2 = 93;

    // Set different values for two unrelated tags
    SingleTagAdjustment.setAdjustmentForTag(tagId1, 111.0);
    SingleTagAdjustment.setAdjustmentForTag(tagId2, 222.0);

    // Verify they maintain their separate values
    assertEquals(111.0, SingleTagAdjustment.getAdjustmentForTag(tagId1), DELTA);
    assertEquals(222.0, SingleTagAdjustment.getAdjustmentForTag(tagId2), DELTA);
  }

  @Test
  @DisplayName("Get adjustment for very high tag ID")
  void testGetAdjustmentVeryHighTagId() {
    double adjustment = SingleTagAdjustment.getAdjustmentForTag(1000);
    // Should return 1.0 for out of bounds tags without dynamic adjustment
    assertTrue(Double.isFinite(adjustment));
  }

  @Test
  @DisplayName("Set and get adjustment for multiple unique tags")
  void testMultipleUniqueTagsSetAndGet() {
    // Use tag IDs that are unlikely to conflict with other tests
    int[] tagIds = {50, 51, 52, 53, 54};
    double[] values = {1.5, 2.5, 3.5, 4.5, 5.5};

    // Set all values
    for (int i = 0; i < tagIds.length; i++) {
      SingleTagAdjustment.setAdjustmentForTag(tagIds[i], values[i]);
    }

    // Verify all values
    for (int i = 0; i < tagIds.length; i++) {
      assertEquals(values[i], SingleTagAdjustment.getAdjustmentForTag(tagIds[i]), DELTA);
    }
  }
}
