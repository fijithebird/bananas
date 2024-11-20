package frc.robot.subsystems.Extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.FeederConstants;

public class ExtensionSim implements ExtensionIO {
    private final FlywheelSim extensionSim =
    new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.00363458292);
  private final PIDController extensionController =
      new PIDController(ExtensionConstants.ExtensionMotorConfig.Slot0.kP, ExtensionConstants.ExtensionMotorConfig.Slot0.kI, ExtensionConstants.ExtensionMotorConfig.Slot0.kD);


  private double AppliedVolts = 0.0;
  private Double SetpointRpm = null;
  private double Feedforward = 0.0;
 
  @Override
  public void updateStats(ExtensionIOStats stats) {
      extensionSim.update(Constants.loopPeriodSecs);

      // control to setpoint
      //if (SetpointRpm != null && SetpointRpm != 0.0) {
      if (SetpointRpm != null) {    
          double av = extensionSim.getAngularVelocityRPM();
          double volt = extensionController.calculate(av, SetpointRpm)+ Feedforward;            
          runVolts(volt);
      }

      stats.PositionRads +=
          Units.radiansToRotations(extensionSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
      stats.VelocityRpm = extensionSim.getAngularVelocityRPM();
      stats.AppliedVolts = AppliedVolts;
      stats.SupplyCurrentAmps = extensionSim.getCurrentDrawAmps();  
  }



 
  public void runVolts(double Volts) {
      SetpointRpm = null;
      //round to 2 decimal places 
      AppliedVolts = MathUtil.clamp(Volts, -12.0, 12.0);
      extensionSim.setInputVoltage(AppliedVolts);
  }

  
  public void extension(double Rpm) {
      SetpointRpm = Rpm;
      this.Feedforward = Feedforward;
  }

  public void setPID(double kP, double kI, double kD) {
      extensionController.setPID(kP, kI, kD);
  }


  public void stop() {
      runVolts(0.0);
  }


  public void runCharacterization(double input) {
      SetpointRpm = null;
      runVolts(input);
  }




@Override
public void runVelocity(double motorRpm, double Feedforward) {
    SetpointRpm = motorRpm;
    this.Feedforward = Feedforward;
}

}

