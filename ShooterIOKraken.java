package frc.robot.subsystems.Shooter;
import javax.swing.text.Position;

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
import edu.wpi.first.units.Velocity;

public class ShooterIOKraken  implements ShooterIO {
  // Hardware
  private final TalonFX shooter1;
  private final TalonFX shooter2;

  // Status Signals
  private final StatusSignal<Double> PositionLeft;
  private final StatusSignal<Double> VelocityLeft;
  private final StatusSignal<Double> AppliedVoltsLeft;
  private final StatusSignal<Double> SupplyCurrentLeft;
  private final StatusSignal<Double> TorqueCurrentLeft;
  private final StatusSignal<Double> TempCelsiusLeft;

  private final StatusSignal<Double> PositionRight; // im here
  private final StatusSignal<Double> VelocityRight;
  private final StatusSignal<Double> AppliedVoltsRight;
  private final StatusSignal<Double> SupplyCurrentRight;
  private final StatusSignal<Double> TorqueCurrentRight;
  private final StatusSignal<Double> TempCelsiusRight;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public ShooterIOKraken() {
    shooter1 = new TalonFX(ShooterConstants.shooter1ID, "drivetrain");
    shooter2 = new TalonFX(ShooterConstants.shooter2ID, "drivetrain");
      

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    //*Cory Added Does not apply to Torque control on Voltage/Velocity Control*/
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true; 
    //*End of Cory Added  */
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.kReduction;

    // Controller config;
    controllerConfig.kP = ShooterConstants.gains.kP();
    controllerConfig.kI = ShooterConstants.gains.kI();
    controllerConfig.kD = ShooterConstants.gains.kD();
    controllerConfig.kS = ShooterConstants.gains.kS();
    controllerConfig.kV = ShooterConstants.gains.kV();
    controllerConfig.kA = ShooterConstants.gains.kA();

    shooter2.setControl(new Follower(shooter1.getDeviceID(), true));

    // Apply configs
    shooter1.getConfigurator().apply(config, 1.0);
    shooter1.getConfigurator().apply(controllerConfig, 1.0);
    shooter2.getConfigurator().apply(config, 1.0);
    shooter2.getConfigurator().apply(controllerConfig, 1.0);

    // Set inverts
    shooter1.setInverted(true);


    // Set signals
    PositionLeft = shooter1.getPosition();
    VelocityLeft = shooter1.getVelocity();
    AppliedVoltsLeft = shooter1.getMotorVoltage();
    SupplyCurrentLeft = shooter1.getSupplyCurrent();
    TorqueCurrentLeft = shooter1.getTorqueCurrent();
    TempCelsiusLeft = shooter1.getDeviceTemp();

    PositionRight = shooter2.getPosition();
    VelocityRight = shooter2.getVelocity();
    AppliedVoltsRight = shooter2.getMotorVoltage();
    SupplyCurrentRight = shooter2.getSupplyCurrent();
    TorqueCurrentRight = shooter2.getTorqueCurrent();
    TempCelsiusRight = shooter2.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        PositionLeft,
        VelocityLeft,
        AppliedVoltsLeft,
        SupplyCurrentLeft,
        TorqueCurrentLeft,
        TempCelsiusLeft,
        
        PositionRight,
        VelocityRight,
        AppliedVoltsRight,
        SupplyCurrentRight,
        TorqueCurrentRight,
        TempCelsiusRight);
  }

  @Override
  public void updateStats(ShooterIOStats stats) {
    stats.MotorConnectedLeft =
        BaseStatusSignal.refreshAll(
          PositionLeft,
          VelocityLeft,
          AppliedVoltsLeft,
          SupplyCurrentLeft,
          TorqueCurrentLeft,
          TempCelsiusLeft)
            .isOK();

    stats.MotorConnectedRight =
        BaseStatusSignal.refreshAll( 
          PositionRight,
          VelocityRight,
          AppliedVoltsRight,
          SupplyCurrentRight,
          TorqueCurrentRight,
          TempCelsiusRight)
            .isOK();

    stats.PositionRadsLeft = Units.rotationsToRadians(PositionLeft.getValueAsDouble());
    stats.VelocityRpmLeft = VelocityLeft.getValueAsDouble() * 60.0;
    stats.AppliedVoltsLeft = AppliedVoltsLeft.getValueAsDouble();
    stats.SupplyCurrentAmpsLeft = SupplyCurrentLeft.getValueAsDouble();
    stats.TorqueCurrentAmpsLeft = TorqueCurrentLeft.getValueAsDouble();
    stats.TempCelsiusLeft = TempCelsiusLeft.getValueAsDouble();

    stats.PositionRadsRight = Units.rotationsToRadians(PositionRight.getValueAsDouble());
    stats.VelocityRpmRight = VelocityRight.getValueAsDouble() * 60.0;
    stats.AppliedVoltsRight = AppliedVoltsRight.getValueAsDouble();
    stats.SupplyCurrentAmpsRight = SupplyCurrentRight.getValueAsDouble();
    stats.TorqueCurrentAmpsRight = TorqueCurrentRight.getValueAsDouble();
    stats.TempCelsiusRight = TempCelsiusRight.getValueAsDouble();

  }

  @Override
  public void runVolts(double Volts) {
    shooter1.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    shooter1.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double Rpm, double Feedforward) {
    shooter1.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
  }

  @Override
  public void setPID(double kP, double kS, double kV) {
    controllerConfig.kP = kP;
    controllerConfig.kS = kS;
    controllerConfig.kV = kV;
    shooter1.getConfigurator().apply(controllerConfig, 1.0);
    shooter2.getConfigurator().apply(controllerConfig, 1.0);
  }

  @Override
  public void runCharacterization(double input) {
    shooter1.setControl(voltageControl.withOutput(input));
  }

}