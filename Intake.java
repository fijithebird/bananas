package frc.robot.subsystems.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOStats;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOStats stats = new IntakeIOStats();
  private final ShuffleboardTab intakeShuffleboard;

  /* Shuffleboard entrys */
  private GenericEntry appliedVoltsInner;
  private GenericEntry velocityRpmInner;
  private GenericEntry positionInner;
  private GenericEntry supplyCurrentInner;
  private GenericEntry statorCurrentInner;
  private GenericEntry tempInner;

  private GenericEntry appliedVoltsOuter;
  private GenericEntry velocityRpmOuter;
  private GenericEntry positionOuter;
  private GenericEntry supplyCurrentOuter;
  private GenericEntry statorCurrentOuter;
  private GenericEntry tempOuter;

  private GenericEntry stateName;


  public enum State {
    IDLE(0.0),
    INTAKE(IntakeConstants.targetRPMHigh),
    CHARACTERIZING(IntakeConstants.targetRPMLow),
    EJECT(-IntakeConstants.targetRPMEject);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /** Creates a new IntakeSubsystem. */
  public Intake(IntakeIO io) {
    this.io = io; 

    this.intakeShuffleboard = Shuffleboard.getTab("Intake");

    appliedVoltsInner = this.intakeShuffleboard.add("Intake Inner Volts", 0.0).getEntry();
    velocityRpmInner = this.intakeShuffleboard.add("Intake RPM Inner", 0.0).getEntry();
    positionInner = this.intakeShuffleboard.add("Intake Pos Inner", 0.0).getEntry();
    supplyCurrentInner = this.intakeShuffleboard.add("Intake Supply Current Inner", 0.0).getEntry();
    statorCurrentInner = this.intakeShuffleboard.add("Intake Stator Current Inner", 0.0).getEntry();
    tempInner = this.intakeShuffleboard.add("Intake Temp Inner", 0.0).getEntry();


    appliedVoltsOuter = this.intakeShuffleboard.add("Intake Outer Volts", 0.0).getEntry();
    velocityRpmOuter = this.intakeShuffleboard.add("Intake RPM Outer", 0.0).getEntry();
    positionOuter = this.intakeShuffleboard.add("Intake Pos Outer", 0.0).getEntry();
    supplyCurrentOuter = this.intakeShuffleboard.add("Intake Supply Current Outer", 0.0).getEntry();
    statorCurrentOuter = this.intakeShuffleboard.add("Intake Stator Current Outer", 0.0).getEntry();
    tempOuter = this.intakeShuffleboard.add("Intake Temp Outer", 0.0).getEntry();


    stateName = this.intakeShuffleboard.add("Intake State", this.currentState.name()).getEntry();
  }

  private void flipState(State inState ) {
   System.out.println("Setting state...." + inState.name());
   currentState = inState; 
  }

  /**
   * Set command to Intake
   *
   * @return a command
   */
  public Command intakeCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.INTAKE);
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
    io.runVelocity(motorRPM, motorRPM, 0.0); 
    //go update the signal data 
    io.updateStats(stats);
    
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
  }

  private void UpdateTelemetry() {
    appliedVoltsInner.setDouble(stats.AppliedVoltsInner);
    velocityRpmInner.setDouble(stats.VelocityRpmInner);
    positionInner.setDouble(stats.PositionRadsInner);
    supplyCurrentInner.setDouble(stats.SupplyCurrentAmpsInner);
    statorCurrentInner.setDouble(stats.TorqueCurrentAmpsInner);

    appliedVoltsOuter.setDouble(stats.AppliedVoltsOuter);
    velocityRpmOuter.setDouble(stats.VelocityRpmOuter);
    positionOuter.setDouble(stats.PositionRadsOuter);
    supplyCurrentOuter.setDouble(stats.SupplyCurrentAmpsOuter);
    statorCurrentOuter.setDouble(stats.TorqueCurrentAmpsOuter);

    stateName.setString(currentState.name());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
