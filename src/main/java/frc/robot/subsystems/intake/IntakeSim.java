package frc.robot.subsystems.intake;

public class IntakeSim implements IntakeIO {

  // Creates a new IntakeSim.
  public IntakeSim() {

    // Registers the subsystem on the command scheduler
    register();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void deploySlapdown() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'deploy'");
  }

  @Override
  public void brakeSlapdown() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'brakeSlapdown'");
  }

  @Override
  public void kindaIn() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'kindaIn'");
  }

  @Override
  public void fullyIn() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'fullyIn'");
  }

  @Override
  public void startRoller() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'startRoller'");
  }

  @Override
  public void stopRoller() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopRoller'");
  }

  @Override
  public boolean withinTolerance(double target) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'withinTolerance'");
  }
}
