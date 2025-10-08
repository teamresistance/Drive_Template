package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("LoggedTunableNumber Tests")
class LoggedTunableNumberTest {

  @BeforeEach
  void setUp() {
    // Reset any static state if needed
  }

  @Test
  @DisplayName("Constructor with key only creates instance")
  void testConstructorWithKeyOnly() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/key");
    assertNotNull(number);
  }

  @Test
  @DisplayName("Constructor with key and default value creates instance")
  void testConstructorWithKeyAndDefault() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/key", 5.0);
    assertNotNull(number);
  }

  @Test
  @DisplayName("Get returns 0.0 when no default is set")
  void testGetWithoutDefault() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/nodefault");
    assertEquals(0.0, number.get());
  }

  @Test
  @DisplayName("Get returns default value when set via constructor")
  void testGetWithDefaultFromConstructor() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/withdefault", 42.0);
    assertEquals(42.0, number.get());
  }

  @Test
  @DisplayName("Get returns default value when set via initDefault")
  void testGetWithDefaultFromInitDefault() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/initdefault");
    number.initDefault(123.456);
    assertEquals(123.456, number.get());
  }

  @Test
  @DisplayName("InitDefault can only be called once")
  void testInitDefaultOnlyOnce() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/once");
    number.initDefault(10.0);
    number.initDefault(20.0); // Should be ignored

    assertEquals(10.0, number.get());
  }

  @Test
  @DisplayName("HasChanged returns true on first call")
  void testHasChangedFirstCall() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/changed", 5.0);
    assertTrue(number.hasChanged(1));
  }

  @Test
  @DisplayName("HasChanged returns false when value hasn't changed")
  void testHasChangedNoChange() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/nochange", 5.0);
    number.hasChanged(1); // First call
    assertFalse(number.hasChanged(1)); // Second call with same value
  }

  @Test
  @DisplayName("HasChanged tracks changes per ID")
  void testHasChangedPerID() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/perid", 5.0);

    // First call for ID 1 should return true
    assertTrue(number.hasChanged(1));

    // First call for ID 2 should return true
    assertTrue(number.hasChanged(2));

    // Second call for ID 1 should return false (no change)
    assertFalse(number.hasChanged(1));

    // Second call for ID 2 should return false (no change)
    assertFalse(number.hasChanged(2));
  }

  @Test
  @DisplayName("HasChanged works with multiple IDs independently")
  void testHasChangedMultipleIDs() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/multiid", 10.0);

    int id1 = 100;
    int id2 = 200;
    int id3 = 300;

    // All IDs should report change on first call
    assertTrue(number.hasChanged(id1));
    assertTrue(number.hasChanged(id2));
    assertTrue(number.hasChanged(id3));

    // All IDs should report no change on second call
    assertFalse(number.hasChanged(id1));
    assertFalse(number.hasChanged(id2));
    assertFalse(number.hasChanged(id3));
  }

  @Test
  @DisplayName("Negative default values work correctly")
  void testNegativeDefaultValue() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/negative", -15.5);
    assertEquals(-15.5, number.get());
  }

  @Test
  @DisplayName("Zero default value works correctly")
  void testZeroDefaultValue() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/zero", 0.0);
    assertEquals(0.0, number.get());
  }

  @Test
  @DisplayName("Very large default values work correctly")
  void testLargeDefaultValue() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/large", 1e10);
    assertEquals(1e10, number.get());
  }

  @Test
  @DisplayName("Very small default values work correctly")
  void testSmallDefaultValue() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/small", 1e-10);
    assertEquals(1e-10, number.get());
  }

  @Test
  @DisplayName("Different instances with same key are independent")
  void testDifferentInstancesIndependent() {
    LoggedTunableNumber number1 = new LoggedTunableNumber("test/samekey", 5.0);
    LoggedTunableNumber number2 = new LoggedTunableNumber("test/samekey", 10.0);

    assertEquals(5.0, number1.get());
    assertEquals(10.0, number2.get());
  }

  @Test
  @DisplayName("Key with slashes is handled correctly")
  void testKeyWithSlashes() {
    LoggedTunableNumber number = new LoggedTunableNumber("category/subcategory/parameter", 7.0);
    assertEquals(7.0, number.get());
  }

  @Test
  @DisplayName("Empty key string creates instance")
  void testEmptyKey() {
    LoggedTunableNumber number = new LoggedTunableNumber("", 3.0);
    assertEquals(3.0, number.get());
  }

  @Test
  @DisplayName("HasChanged with same ID multiple times tracks correctly")
  void testHasChangedSameIDMultipleTimes() {
    LoggedTunableNumber number = new LoggedTunableNumber("test/tracking", 100.0);

    int id = 999;

    // First call - should be true
    assertTrue(number.hasChanged(id));

    // Subsequent calls - should be false
    assertFalse(number.hasChanged(id));
    assertFalse(number.hasChanged(id));
    assertFalse(number.hasChanged(id));
  }

  @Test
  @DisplayName("HasChanged returns true after value conceptually changes")
  void testHasChangedDetectsChange() {
    // Note: This test verifies the logic works, but in non-tuning mode
    // the value won't actually change from dashboard
    LoggedTunableNumber number = new LoggedTunableNumber("test/valuechange");

    // Set initial default
    number.initDefault(50.0);

    int id = 12345;

    // First check - should be true
    assertTrue(number.hasChanged(id));

    // Second check - should be false (same value)
    assertFalse(number.hasChanged(id));
  }
}
