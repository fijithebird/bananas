package frc.robot.subsystems.Extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Extension.ExtensionConstants;

public class ExtensionKraken implements ExtensionIO {

    private final TalonFX extensionTalon;
    
    private final StatusSignal<Double> Position;
    private final StatusSignal<Double> Velocity;
    private final StatusSignal<Double> AppliedVolts;
    private final StatusSignal<Double> SupplyCurrent;
    private final StatusSignal<Double> TorqueCurrent;
    private final StatusSignal<Double> TempCelsius;

    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
   // private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
 
    public ExtensionKraken(){
    extensionTalon = new TalonFX(ExtensionConstants.kExtensionMotorID, ExtensionConstants.CANBUS_NAME);
    
    extensionTalon.getConfigurator().apply(ExtensionConstants.ExtensionMotorConfig, 1.0);
   
    extensionTalon.setInverted(false);
    

    Position = extensionTalon.getPosition();
    Velocity = extensionTalon.getPosition();
    AppliedVolts = extensionTalon.getMotorVoltage();
    SupplyCurrent = extensionTalon.getSupplyCurrent();
    TorqueCurrent = extensionTalon.getTorqueCurrent();
    TempCelsius = extensionTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        Position,
        Velocity,
        AppliedVolts,
        SupplyCurrent,
        TorqueCurrent);
  }
  @Override
  public void updateStats(ExtensionIOStats stats){
    stats.MotorConnected = 
    BaseStatusSignal.refreshAll(
            Position,
            Velocity,
            AppliedVolts,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius)
        .isOK();

        stats.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
        stats.VelocityRpm = Velocity.getValueAsDouble() * 60.0;
        stats.AppliedVolts = AppliedVolts.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();
       
     }
    
     public void runVolts(double Volts) {
       extensionTalon.setControl(voltageControl.withOutput(Volts));
     }
   
     
     public void stop() {
       extensionTalon.setControl(neutralControl);
     }
   
    
     /**
      * @param kp P in PIDs
      * @param kV Amperage needed to sustain a high setpoint (kinda)
      * @param kS Amperage needed to overcome static friction (kinda)
      * kv ks and kp are the only values needed to get a good tune on any motor to attain a velocity
      */
     public void setPID(double kP, double kV, double kS) {
        extensionTalon.getConfigurator().apply(new Slot0Configs()
         .withKP(kP)
         .withKV(kV)
         .withKS(kS));
     }
   
     public void runCharacterization(double input) {
       extensionTalon.setControl(voltageControl.withOutput(input));
     }
     public void runVelocity(double motorRpm, double Feedforward) {
      if (motorRpm != 0) {
        extensionTalon.setControl(
          velocityControl.withVelocity(motorRpm / 60.0));
      } else {
        extensionTalon.setControl(neutralControl);
      }
    }

    public void extend(double Rpm) {
   
  
      if (Rpm != 0) {
        extensionTalon.setControl(
          velocityControl.withVelocity(Rpm / 60.0));
      } else {
        extensionTalon.setControl(neutralControl);
      }
  
    }
}
