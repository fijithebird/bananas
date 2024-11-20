package frc.robot.subsystems.Extension;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import frc.robot.subsystems.Extension.ExtensionIO;
//import frc.robot.subsystems.Extension.ExtensionIO.ExtensionIOStats;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Extension.ExtensionIO.ExtensionIOStats;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
public class Extension extends SubsystemBase  {

  private final ExtensionIO io;
  private final ExtensionIOStats stats = new ExtensionIOStats();
  private final ShuffleboardTab extensionShuffleboard;

 private GenericEntry appliedVolts;
  private GenericEntry velocityRpm;
  private GenericEntry position;
  private GenericEntry supplyCurrent;
  private GenericEntry statorCurrent;
  private GenericEntry stateName;

  private static final double _ExtendRPM = 3500.0;
  private static final double _Idle = 3500.0; 



    public enum State{
  
        IDLE(_Idle),
        EXTEND(_ExtendRPM);

        private final double rpm; 

        private State (double rpmIn) {
          this.rpm = rpmIn; 
    }
  }
    private State currentState = State.IDLE;



    public Extension(ExtensionIO io) {
    this.io = io;
    this.extensionShuffleboard = Shuffleboard.getTab("Extension");
    position = this.extensionShuffleboard.add("extension Position", 0.0).getEntry();
    appliedVolts = this.extensionShuffleboard.add("extension Volts", 0.0).getEntry();
    velocityRpm = this.extensionShuffleboard.add("extension RPM ", 0.0).getEntry();
    supplyCurrent = this.extensionShuffleboard.add("extension Supply Current", 0.0).getEntry();
    statorCurrent = this.extensionShuffleboard.add("extension Stator Current", 0.0).getEntry();
    stateName = this.extensionShuffleboard.add("Extension State", currentState.name()).getEntry();
  }

  public void flipState(State inState) {
    System.out.println("Setting extension state...." + inState.name());
   currentState = inState; 
  }
    
   public Command idleCommand(){
       return runOnce(() -> {currentState = State.IDLE;});
      
   }
   public Command extensionCommand() {
    
     return runOnce(() -> {currentState = State.EXTEND;});
   }
  


 
public void periodic(){
  double motorRPM = currentState.rpm;
  if(currentState == State.IDLE || currentState == State.EXTEND){
   io.runVelocity(motorRPM,motorRPM);
   UpdateTelemetry();
  }
  }


   // io.updateStats(stats);
  // UpdateTelemetry();
  //}
  private void UpdateTelemetry() {
    appliedVolts.setDouble(stats.AppliedVolts);
    velocityRpm.setDouble(stats.VelocityRpm);
    position.setDouble(stats.PositionRads);
    supplyCurrent.setDouble(stats.SupplyCurrentAmps);
    statorCurrent.setDouble(stats.TorqueCurrentAmps);
    stateName.setString(currentState.name());
   // temp.setDouble(stats.TempCelsius);

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
