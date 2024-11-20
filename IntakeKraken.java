package frc.robot.subsystems.Intake;

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

public class IntakeKraken implements IntakeIO {
  // Hardware
  private final TalonFX intakeTalonInner;
  private final TalonFX intakeTalonOuter;


  // Status Signals
  private final StatusSignal<Double> PositionInner;
  private final StatusSignal<Double> VelocityInner;
  private final StatusSignal<Double> AppliedVoltsInner;
  private final StatusSignal<Double> SupplyCurrentInner;
  private final StatusSignal<Double> TorqueCurrentInner;
  private final StatusSignal<Double> TempCelsiusInner;

  private final StatusSignal<Double> PositionOuter;
  private final StatusSignal<Double> VelocityOuter;
  private final StatusSignal<Double> AppliedVoltsOuter;
  private final StatusSignal<Double> SupplyCurrentOuter;
  private final StatusSignal<Double> TorqueCurrentOuter;
  private final StatusSignal<Double> TempCelsiusOuter;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public IntakeKraken() {
    intakeTalonInner = new TalonFX(IntakeConstants.kInnerIntakeMotorID, IntakeConstants.CANBUS_NAME);
    intakeTalonOuter = new TalonFX(IntakeConstants.kOuterIntakeMotorID, IntakeConstants.CANBUS_NAME);


    // Apply configs
    intakeTalonInner.getConfigurator().apply(IntakeConstants.innerRollerConfig, 1.0);
    intakeTalonOuter.getConfigurator().apply(IntakeConstants.outerRollerConfig, 1.0);

    // Set inverts
    intakeTalonInner.setInverted(false);
    intakeTalonOuter.setInverted(true);

    // Set signals
    PositionInner = intakeTalonInner.getPosition();
    VelocityInner = intakeTalonInner.getVelocity();
    AppliedVoltsInner = intakeTalonInner.getMotorVoltage();
    SupplyCurrentInner = intakeTalonInner.getSupplyCurrent();
    TorqueCurrentInner = intakeTalonInner.getTorqueCurrent();
    TempCelsiusInner = intakeTalonInner.getDeviceTemp();

    PositionOuter = intakeTalonOuter.getPosition();
    VelocityOuter = intakeTalonOuter.getVelocity();
    AppliedVoltsOuter = intakeTalonOuter.getMotorVoltage();
    SupplyCurrentOuter = intakeTalonOuter.getSupplyCurrent();
    TorqueCurrentOuter = intakeTalonOuter.getTorqueCurrent();
    TempCelsiusOuter = intakeTalonOuter.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        PositionInner,
        VelocityInner,
        AppliedVoltsInner,
        SupplyCurrentInner,
        TorqueCurrentInner,
        TempCelsiusInner,
        
        PositionOuter,
        VelocityOuter,
        AppliedVoltsOuter,
        SupplyCurrentOuter,
        TorqueCurrentOuter,
        TempCelsiusOuter);
  }

  @Override
  public void updateStats(IntakeIOStats stats) {
    stats.MotorConnectedInner =
        BaseStatusSignal.refreshAll(
          PositionInner,
          VelocityInner,
          AppliedVoltsInner,
          SupplyCurrentInner,
          TorqueCurrentInner,
          TempCelsiusInner)
            .isOK();

    stats.MotorConnectedOuter =
        BaseStatusSignal.refreshAll( 
          PositionOuter,
          VelocityOuter,
          AppliedVoltsOuter,
          SupplyCurrentOuter,
          TorqueCurrentOuter,
          TempCelsiusOuter)
            .isOK();

    stats.PositionRadsInner = Units.rotationsToRadians(PositionInner.getValueAsDouble());
    stats.VelocityRpmInner = VelocityInner.getValueAsDouble() * 60.0;
    stats.AppliedVoltsInner = AppliedVoltsInner.getValueAsDouble();
    stats.SupplyCurrentAmpsInner = SupplyCurrentInner.getValueAsDouble();
    stats.TorqueCurrentAmpsInner = TorqueCurrentInner.getValueAsDouble();
    stats.TempCelsiusInner = TempCelsiusInner.getValueAsDouble();

    stats.PositionRadsOuter = Units.rotationsToRadians(PositionOuter.getValueAsDouble());
    stats.VelocityRpmOuter = VelocityOuter.getValueAsDouble() * 60.0;
    stats.AppliedVoltsOuter = AppliedVoltsOuter.getValueAsDouble();
    stats.SupplyCurrentAmpsOuter = SupplyCurrentOuter.getValueAsDouble();
    stats.TorqueCurrentAmpsOuter = TorqueCurrentOuter.getValueAsDouble();
    stats.TempCelsiusOuter = TempCelsiusOuter.getValueAsDouble();

  }

  @Override
  public void runVolts(double Volts) {
    intakeTalonInner.setControl(voltageControl.withOutput(Volts));
    intakeTalonOuter.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    intakeTalonInner.setControl(neutralControl);
    intakeTalonOuter.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double Rpm, double outerRpm, double Feedforward) {
    if (Rpm != 0) {
      intakeTalonInner.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
      intakeTalonOuter.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
    } else {
      intakeTalonInner.setControl(neutralControl); // because TorqueFOC tuning doesnt like a 0 command
      intakeTalonOuter.setControl(neutralControl);
    }
    
  }

  @Override
  /**
   * @param kp P in PID
   * @param kV Amperage needed to sustain a high setpoint (kinda)
   * @param kS Amperage needed to overcome static friction (kinda)
   * kv ks and kp are the only values needed to get a good tune on any motor to attain a velocity
   */
  public void setPID(double kP, double kV, double kS) {
    intakeTalonInner.getConfigurator().apply(new Slot0Configs()
      .withKP(kP)
      .withKV(kV)
      .withKS(kS));
    intakeTalonOuter.getConfigurator().apply(new Slot0Configs()
      .withKP(kP)
      .withKV(kV)
      .withKS(kS));
  }

  @Override
  public void runCharacterization(double input) {
    intakeTalonInner.setControl(voltageControl.withOutput(input));
    intakeTalonOuter.setControl(voltageControl.withOutput(input));
  }

}
