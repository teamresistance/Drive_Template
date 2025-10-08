# Test Suite

JUnit Jupiter unit tests.

## Test Structure

- `test/java/frc/robot/util/` - Tests for utility classes
- `test/java/frc/robot/subsystems/drive/` - Tests for the drive subsystem
- `test/java/frc/robot/subsystems/vision/` - Tests for the vision subsystem

## Test Files

### Utility Tests

- **GeomUtilTest.java** - Tests for geometry utility methods (22 tests)
  - Translation, rotation, and pose conversion methods
  - 2D and 3D geometry transformations
  - Twist multiplication

- **PolynomialRegressionTest.java** - Tests for polynomial regression (14 tests)
  - Linear, quadratic, and cubic regression
  - R² coefficient calculation
  - Prediction methods
  - Edge cases (constant functions, noisy data)

- **LoggedTunableNumberTest.java** - Tests for tunable parameters (17 tests)
  - Default value initialization
  - Change detection per ID
  - Multiple instance handling

- **TimestampedVisionUpdateTest.java** - Tests for vision update records (15 tests)
  - Record creation and accessors
  - Equality and hashCode
  - Edge cases (zero/negative timestamps)

### Subsystem Tests

- **ModuleTest.java** - Tests for swerve module (24 tests)
  - Module state and position
  - Setpoint optimization
  - Characterization methods
  - Odometry position calculation
  - Uses Mockito for IO mocking

- **SingleTagAdjustmentTest.java** - Tests for AprilTag adjustments (13 tests)
  - Dynamic adjustment factors
  - Tag ID validation
  - Multi-tag isolation

## Running Tests

Run all tests:
```bash
./gradlew test
```

Run tests with detailed output:
```bash
./gradlew test --info
```

View test report:
```bash
open build/reports/tests/test/index.html
```
