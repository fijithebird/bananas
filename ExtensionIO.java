package frc.robot.subsystems.Extension;

public interface ExtensionIO { 
    class ExtensionIOStats {
   
        public double PositionRads = 0.0;
        public double VelocityRpm = 0.0;
        public double AppliedVolts = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
        public boolean MotorConnected;
      }
/** Update stats */
default void updateStats(ExtensionIOStats stats) {}

/** Run motor at voltage */
default void runVolts(double Volts) {}

/** Stop Intake */
default void extend(double Rpm){}
default void stop() {}

/** Run motor at velocity in rpm with breambreak stop*/
default void intake(double Rpm) {}

/** Run motor without caring abt the beambreak */
default void shoot(double Rpm) {}

/** Config PID values for motor */
default void setPID(double kP, double kI, double kD) {}

/** Run Characterization for motor at voltage */
default void runCharacterization(double input) {}

void runVelocity(double motorRPM, double Feedforward);
}
