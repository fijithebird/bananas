package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public interface FeederIO {
    class FeederIOStats {
        public boolean MotorConnected = true;    
        public double PositionRads = 0.0;
        public double VelocityRpm = 0.0;
        public double AppliedVolts = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
      }

/** Update stats */
default void updateStats(FeederIOStats stats) {}

/** Run motor at voltage */
default void runVolts(double Volts) {}

/** Stop Intake */
default void stop() {}

/** Run motor at velocity in rpm with breambreak stop*/
default void intake(double Rpm) {}

/** Run motor without caring abt the beambreak */
default void shoot(double Rpm) {}

/** Config PID values for motor */
default void setPID(double kP, double kI, double kD) {}

/** Run Characterization for motor at voltage */
default void runCharacterization(double input) {}
}
