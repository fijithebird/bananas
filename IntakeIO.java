package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public interface IntakeIO {
    class IntakeIOStats {
        public boolean MotorConnectedInner = true;    
        public double PositionRadsInner = 0.0;
        public double VelocityRpmInner = 0.0;
        public double AppliedVoltsInner = 0.0;
        public double SupplyCurrentAmpsInner = 0.0;
        public double TorqueCurrentAmpsInner = 0.0;
        public double TempCelsiusInner = 0.0;

        public boolean MotorConnectedOuter = true;    
        public double PositionRadsOuter = 0.0;
        public double VelocityRpmOuter = 0.0;
        public double AppliedVoltsOuter = 0.0;
        public double SupplyCurrentAmpsOuter = 0.0;
        public double TorqueCurrentAmpsOuter = 0.0;
        public double TempCelsiusOuter = 0.0;
      }

/** Update stats */
default void updateStats(IntakeIOStats stats) {}

/** Run motor at voltage */
default void runVolts(double Volts) {}

/** Stop Intake */
default void stop() {}

/** Run motor at velocity in rpm */
default void runVelocity(double Rpm, double outerRpm, double Feedforward) {}

/** Config PID values for motor */
default void setPID(double kP, double kI, double kD) {}

/** Config FF values for motor */
default void setFF(double kS, double kV, double kA) {}

/** Run Characterization for motor at voltage */
default void runCharacterization(double input) {}
}
