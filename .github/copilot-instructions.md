# GitHub Copilot Instructions for Team Resistance #86

## Project Structure & Context

### Key Technologies

- **Language**: Java 17
- **Framework**: WPILib 2025
- **Logging**: AdvantageKit
- **Motor Control**: Phoenix 6 (CTRE)
- **Pathfinding**: PathPlanner
- **Vision**: PhotonVision
- **Build Tool**: Gradle

### Package Structure

- `frc.robot` - Main robot code
- `frc.robot.subsystems` - Robot subsystems (drive, vision, etc.)
- `frc.robot.commands` - Robot commands
- `frc.robot.util` - Utility classes
- `frc.robot.generated` - Auto-generated files, usually configuration

## Coding Standards & Best Practices

### Code Style

- Use WPILib Java code conventions
- Follow AdvantageKit patterns for IO abstraction
- Use descriptive variable and method names
- Include comprehensive JavaDoc comments for public methods
- Use proper exception handling for hardware failures

### FRC-Specific Guidelines

- Use the command-based robot framework
- Implement proper subsystem requirements in commands
- Use AdvantageKit's IO layer pattern for hardware abstraction
- Log all important robot data using Logger.recordOutput()
- Handle robot modes (teleop, autonomous, test) appropriately
- Use Units class for all physical measurements

### Hardware Abstraction

- Separate hardware interfaces (IO classes) from subsystem logic
- Create real, simulation, and replay implementations for each IO interface
- Use the pattern: `SubsystemIO`, `SubsystemIOReal`, `SubsystemIOSim`
- Always check for hardware failures and provide fallback behavior

### Safety & Reliability

- Implement software limits for all mechanisms
- Add current limiting to prevent brownouts
- Use Phoenix 6's built-in safety features
- Always validate sensor readings before using them
- Implement graceful degradation when sensors fail

## Domain-Specific Knowledge

### Swerve Drive

- This template uses CTRE Phoenix 6 swerve modules
- Module configuration is in `TunerConstants.java` (generated file)
- Drive commands should use field-relative control by default
- Implement proper wheel angle optimization
- Use odometry for accurate position tracking

### AdvantageKit Integration

- Use `LoggedRobot` instead of `TimedRobot`
- Implement IO interfaces for all hardware interactions
- Log inputs and outputs using `Logger.recordOutput()`
- Support real robot, simulation, and log replay modes
- Use proper metadata logging for build information

### PathPlanner Integration

- Configure path constraints in `Constants.java`
- Use PathPlanner's auto command factories
- Implement proper path following with feedback control
- Store autonomous routines in `src/main/deploy/pathplanner/autos/`

### Vision System

- Integrate PhotonVision for AprilTag detection
- Use vision measurements to correct odometry drift
- Implement proper coordinate frame transformations
- Handle vision measurement rejection for outliers

## Code Generation Guidelines

### When creating new subsystems:

1. Create IO interface with inputs and outputs classes
2. Implement real hardware IO class
3. Implement simulation IO class
4. Create main subsystem class using IO abstraction
5. Add proper logging throughout
6. Include unit tests where applicable

### When creating new commands:

1. Extend `Command` or use factory methods
2. Declare subsystem requirements properly
3. Implement initialize(), execute(), end(), and isFinished()
4. Add logging for command state changes
5. Handle interruption gracefully

### When adding new sensors:

1. Create IO interface following AdvantageKit patterns
2. Implement proper sensor validation
3. Add simulation support
4. Log all sensor data
5. Implement fault detection and reporting

## File Naming Conventions

- Subsystems: `NameSubsystem.java`
- IO Interfaces: `NameIO.java`
- Real implementations: `NameIOReal.java` or `NameIOTalonFX.java`
- Simulation implementations: `NameIOSim.java`
- Commands: `VerbNounCommand.java` or use command factories

## Dependencies & Libraries

- Prefer WPILib built-in classes over third-party alternatives
- Use Phoenix 6 for CTRE devices (TalonFX, CANcoder, Pigeon2)
- Use REVLib for REV devices (SparkMax, etc.)
- Leverage AdvantageKit for all hardware abstraction
- Use PathPlanner for autonomous path planning

## Testing & Validation

- Test code in simulation before deploying to robot
- Use AdvantageScope for data analysis and debugging
- Implement unit tests for critical algorithms
- Validate all hardware configurations before competition
- Use proper error handling and user feedback

## Robot Configuration

- Team number and robot-specific settings in `.wpilib/` folder
- Vendor dependencies in `vendordeps/` folder
- Path files in `src/main/deploy/pathplanner/`
- Tuning constants should be easily accessible and well-documented

Remember: Safety first! Always consider what happens when hardware fails or sensors give bad readings. FRC robots operate in high-stress environments where reliability is crucial.
