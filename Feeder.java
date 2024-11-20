package frc.robot.subsystems.Feeder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;
import frc.robot.subsystems.Feeder.FeederIO.FeederIOStats;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOStats stats = new FeederIOStats();
  private final ShuffleboardTab feederShuffleboard;

  /* Shuffleboard entrys */
  private GenericEntry appliedVolts;
  private GenericEntry velocityRpm;
  private GenericEntry position;
  private GenericEntry supplyCurrent;
  private GenericEntry statorCurrent;
  private GenericEntry temp;

  private GenericEntry stateName;

  public enum State {
    IDLE(0.0),
    FEEDER(FeederConstants.targetRPMHigh),
    SHOOT(FeederConstants.targetRPMHigh),
    CHARACTERIZING(FeederConstants.targetRPMLow),
    EJECT(-FeederConstants.targetRPMEject);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /** Creates a new FeederSubsystem. */
  public Feeder(FeederIO io) {
    this.io = io; 
    this.feederShuffleboard = Shuffleboard.getTab("Feeder");

    appliedVolts = this.feederShuffleboard.add("Feeder Volts", 0.0).getEntry();
    velocityRpm = this.feederShuffleboard.add("Feeder RPM ", 0.0).getEntry();
    position = this.feederShuffleboard.add("Feeder Pos", 0.0).getEntry();
    supplyCurrent = this.feederShuffleboard.add("Feeder Supply Current", 0.0).getEntry();
    statorCurrent = this.feederShuffleboard.add("Feeder Stator Current", 0.0).getEntry();
    temp = this.feederShuffleboard.add("Feeder Temp", 0.0).getEntry();

    stateName = this.feederShuffleboard.add("Feeder State", currentState.name()).getEntry();
  }

  private void flipState(State inState ) {
   System.out.println("Setting feeder state...." + inState.name());
   currentState = inState; 
  }

  /**
   * Set command to Feeder
   *
   * @return a command
   */
  public Command FeederCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.FEEDER);
        });
  }

   /**
   * Set command to Idle
   *
   * @return a command
   */
  public Command idleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.IDLE);
        });
  }

   /**
   * Set Command to Eject
   *
   * @return a command
   */
  public Command ejectCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {flipState(State.EJECT); });
  }

  public Command shoot() {
    return runOnce(() -> {flipState(State.SHOOT); });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    
    //Detwrmine motor speed and process 
    double motorRPM = currentState.rpm;
    if (currentState == State.SHOOT) {
      io.shoot(motorRPM);

    } else if (currentState == State.FEEDER) {
      io.intake(motorRPM); 
    } else {
      io.intake(motorRPM);
    }

    //go update the signal data 
    io.updateStats(stats);
    
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
  }

  private void UpdateTelemetry() {
    appliedVolts.setDouble(stats.AppliedVolts);
    velocityRpm.setDouble(stats.VelocityRpm);
    position.setDouble(stats.PositionRads);
    supplyCurrent.setDouble(stats.SupplyCurrentAmps);
    statorCurrent.setDouble(stats.TorqueCurrentAmps);
    temp.setDouble(stats.TempCelsius);

    stateName.setString(currentState.name());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
