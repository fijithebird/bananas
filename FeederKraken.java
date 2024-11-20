package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class FeederKraken implements FeederIO {
  // Hardware
  private final TalonFX feederTalon;
  private final DigitalInput feederBeamBreak;
  private boolean hasRing;



  // Status Signals
  private final StatusSignal<Double> Position;
  private final StatusSignal<Double> Velocity;
  private final StatusSignal<Double> AppliedVolts;
  private final StatusSignal<Double> SupplyCurrent;
  private final StatusSignal<Double> TorqueCurrent;
  private final StatusSignal<Double> TempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public FeederKraken() {
    feederTalon = new TalonFX(FeederConstants.kFeederMotorID, FeederConstants.CANBUS_NAME);
    feederBeamBreak = new DigitalInput(0);

    // Apply configs
    feederTalon.getConfigurator().apply(FeederConstants.FeederMotorConfig, 1.0);

    // Set inverts
    feederTalon.setInverted(false);

    // Set signals
    Position = feederTalon.getPosition();
    Velocity = feederTalon.getVelocity();
    AppliedVolts = feederTalon.getMotorVoltage();
    SupplyCurrent = feederTalon.getSupplyCurrent();
    TorqueCurrent = feederTalon.getTorqueCurrent();
    TempCelsius = feederTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        Position,
        Velocity,
        AppliedVolts,
        SupplyCurrent,
        TorqueCurrent,
        TempCelsius);

  }

  @Override
  public void updateStats(FeederIOStats stats) {
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

  @Override
  public void runVolts(double Volts) {
    feederTalon.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    feederTalon.setControl(neutralControl);
  }

  @Override
  public void intake(double Rpm) {
    if (Rpm != 0) {

      if (!hasRing && feederBeamBreak.get()) { // first loop it sees the ring
        hasRing = true;
        feederTalon.setPosition(0.0); // reset motor encoder

      }

      if (hasRing && feederTalon.getPosition().getValue() <= FeederConstants.ringAdvance) {
        feederTalon.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
      } else if (!hasRing) {
        feederTalon.setControl(
        velocityControl.withVelocity(Rpm / 60.0)); 
      } else {

        feederTalon.setControl(neutralControl);
      }
      
    } else {
      feederTalon.setControl(neutralControl); // because TorqueFOC tuning doesnt like a 0 command
    }
  }

  @Override
  public void shoot(double Rpm) {
    hasRing = false;

    if (Rpm != 0) {
      feederTalon.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
    } else {
      feederTalon.setControl(neutralControl);
    }

  }

  @Override
  /**
   * @param kp P in PIDs
   * @param kV Amperage needed to sustain a high setpoint (kinda)
   * @param kS Amperage needed to overcome static friction (kinda)
   * kv ks and kp are the only values needed to get a good tune on any motor to attain a velocity
   */
  public void setPID(double kP, double kV, double kS) {
    feederTalon.getConfigurator().apply(new Slot0Configs()
      .withKP(kP)
      .withKV(kV)
      .withKS(kS));
  }

  @Override
  public void runCharacterization(double input) {
    feederTalon.setControl(voltageControl.withOutput(input));
  }

}
