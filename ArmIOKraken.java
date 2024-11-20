package frc.robot.subsystems.Arm;
import javax.swing.text.Position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Velocity;

public class ArmIOKraken  implements ArmIO {


TalonFX armMotor;
TalonFX armMotorFollower;

MotionMagicVoltage armMagic;

CANcoder cancoder;

private final StatusSignal<Boolean> f_fusedSensorOutOfSync;
private final StatusSignal<Boolean> sf_fusedSensorOutOfSync;
private final StatusSignal<Boolean> f_remoteSensorInvalid;
private final StatusSignal<Boolean> sf_remoteSensorInvalid;

private final StatusSignal<Double> armPosition;
private final StatusSignal<Double> armVelocity;
private final StatusSignal<Double> armRotorPos;
private final StatusSignal<Double> cancoderPosition;
private final StatusSignal<Double> cancoderVelocity;

private final StatusSignal<Double> armFollowerPosition;
private final StatusSignal<Double> armFollowerVelocity;
private final StatusSignal<Double> armFollowerRotorPos;

private final StatusSignal<Double> SupplyCurrentLeader;
private final StatusSignal<Double> TorqueCurrentLeader;
private final StatusSignal<Double> TempCelsiusLeader;

private final StatusSignal<Double> SupplyCurrentFollower;
private final StatusSignal<Double> TorqueCurrentFollower;
private final StatusSignal<Double> TempCelsiusFollower;


  public ArmIOKraken() {
    armMotor = new TalonFX(ArmConstants.arm1MotorID, "drivetrain");
    armMotorFollower = new TalonFX(ArmConstants.arm2MotorID, "drivetrain");
    cancoder = new CANcoder(44, "drivetrain");
      
    armMagic = new MotionMagicVoltage(0);
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = ArmConstants.armGains.CruiseVelocity(); //rps
    mm.MotionMagicAcceleration = ArmConstants.armGains.Acceleration();
    mm.MotionMagicJerk = ArmConstants.armGains.Jerk();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = ArmConstants.armGains.kP();
    slot0.kI = ArmConstants.armGains.kI();
    slot0.kD = ArmConstants.armGains.kD();
    slot0.kV = ArmConstants.armGains.kV();
    slot0.kS = ArmConstants.armGains.kS(); // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1;

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.023;
    cancoder.getConfigurator().apply(cancoderConfig);

    cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
    cfg.Feedback.RotorToSensorRatio = 1; //12.8;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .5;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .25;


    StatusCode armStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      armStatus = armMotor.getConfigurator().apply(cfg);
      if (armStatus.isOK()) break;
    }
    if (!armStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + armStatus.toString());
    }

    StatusCode armStatus2 = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      armStatus2 = armMotorFollower.getConfigurator().apply(cfg);
      if (armStatus2.isOK()) break;
    }
    if (!armStatus2.isOK()) {
      System.out.println("Could not configure device. Error: " + armStatus2.toString());
    }

    armMotorFollower.setInverted(true);

    

    f_fusedSensorOutOfSync = armMotor.getFault_FusedSensorOutOfSync();
    sf_fusedSensorOutOfSync = armMotor.getStickyFault_FusedSensorOutOfSync();
    f_remoteSensorInvalid = armMotor.getFault_RemoteSensorDataInvalid();
    sf_remoteSensorInvalid = armMotor.getStickyFault_RemoteSensorDataInvalid();

    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    armRotorPos = armMotor.getRotorPosition();
    cancoderPosition = cancoder.getPosition();
    cancoderVelocity = cancoder.getVelocity();

    armFollowerPosition = armMotor.getPosition();
    armFollowerVelocity = armMotor.getVelocity();
    armFollowerRotorPos = armMotor.getRotorPosition();

    SupplyCurrentLeader = armMotor.getSupplyCurrent();
    TorqueCurrentLeader = armMotor.getTorqueCurrent();
    TempCelsiusLeader = armMotor.getDeviceTemp();

    SupplyCurrentFollower = armMotorFollower.getSupplyCurrent();
    TorqueCurrentFollower = armMotorFollower.getTorqueCurrent();
    TempCelsiusFollower = armMotorFollower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        f_fusedSensorOutOfSync,
        sf_fusedSensorOutOfSync,
        f_remoteSensorInvalid,
        sf_remoteSensorInvalid,
        armPosition,
        armVelocity,
        cancoderPosition,
        cancoderVelocity,
        armRotorPos,
        armFollowerPosition,
        armFollowerVelocity,
        armFollowerRotorPos,
        SupplyCurrentLeader,
        TorqueCurrentLeader,
        TempCelsiusLeader,
        SupplyCurrentFollower,
        TorqueCurrentFollower,
        TempCelsiusFollower
      );
  }

  @Override
  public void updateArmStats(ArmIOStats stats) {
    stats.armMotorConnected =
        BaseStatusSignal.refreshAll(
          f_fusedSensorOutOfSync,
          sf_fusedSensorOutOfSync,
          f_remoteSensorInvalid,
          sf_remoteSensorInvalid,
          armPosition,
          armVelocity,
          cancoderPosition,
          cancoderVelocity,
          armRotorPos,
          armFollowerPosition,
          armFollowerVelocity,
          armFollowerRotorPos,
          SupplyCurrentLeader,
          TorqueCurrentLeader,
          TempCelsiusLeader,
          SupplyCurrentFollower,
          TorqueCurrentFollower,
          TempCelsiusFollower)
            .isOK();

    stats.cancoderPosition = cancoderPosition.getValueAsDouble();
    stats.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    stats.armPosition = armPosition.getValueAsDouble();
    stats.armVelocity = armVelocity.getValueAsDouble();

    stats.armFollowerPosition = armFollowerPosition.getValueAsDouble();
    stats.armFollowerVelocity = armFollowerVelocity.getValueAsDouble();

    stats.SupplyCurrentAmpsLeader = SupplyCurrentLeader.getValueAsDouble();
    stats.TorqueCurrentAmpsLeader = TorqueCurrentLeader.getValueAsDouble();
    stats.TempCelsiusLeader = TempCelsiusLeader.getValueAsDouble();

    stats.SupplyCurrentAmpsFollower = SupplyCurrentFollower.getValueAsDouble();
    stats.TorqueCurrentAmpsFollower = TorqueCurrentFollower.getValueAsDouble();
    stats.TempCelsiusFollower = TempCelsiusFollower.getValueAsDouble();
  }

  @Override
  public void setArmMotorControl(double commandedPosition){
  armMotor.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
  armMotorFollower.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
  }

}