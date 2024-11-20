package frc.robot.subsystems.Extension;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import edu.wpi.first.math.util.Units;
//import java.lang.Math;
public class ExtensionConstants {
    public static final double ExtendRPM = 3000; // any number from -6000 to 6000
    public static final double targetRPMLow = 3000;
    public static final double targetRPMEject = 1800;
    public static final int kExtensionMotorID = 14;
    public static final String CANBUS_NAME = "drivetrain";
    
    public static final TalonFXConfiguration ExtensionMotorConfig = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs() // current limits for voltage based control
    .withStatorCurrentLimit(40)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentThreshold(50)
    .withSupplyTimeThreshold(0.0)

).withTorqueCurrent(new TorqueCurrentConfigs() // current limits for torqueFOC based control
    .withPeakForwardTorqueCurrent(40)
    .withPeakReverseTorqueCurrent(-40)

).withSlot0(new Slot0Configs()  // PID gains, more slots can be added for differrent current limits / senarios
    .withKP(2.0)
    .withKI(0.0)
    .withKD(0.0)
    .withKS(0.0)
    .withKS(3.5)
    .withKV(0.0)
    .withKA(0.0)

).withMotorOutput(new MotorOutputConfigs()
    .withInverted(InvertedValue.CounterClockwise_Positive)
    .withNeutralMode(NeutralModeValue.Brake)

).withFeedback(new FeedbackConfigs()
    .withSensorToMechanismRatio(0.0)
);
}
