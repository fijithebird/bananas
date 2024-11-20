package frc.robot.subsystems.Shooter;

public interface ShooterIO {
        
        class ShooterIOStats {
                public boolean MotorConnectedLeft = true;    
                public double PositionRadsLeft = 0.0;
                public double VelocityRpmLeft = 0.0;
                public double AppliedVoltsLeft = 0.0;
                public double SupplyCurrentAmpsLeft = 0.0;
                public double TorqueCurrentAmpsLeft = 0.0;
                public double TempCelsiusLeft = 0.0;


                public boolean MotorConnectedRight = true;    
                public double PositionRadsRight = 0.0;
                public double VelocityRpmRight = 0.0;
                public double AppliedVoltsRight = 0.0;
                public double SupplyCurrentAmpsRight = 0.0;
                public double TorqueCurrentAmpsRight = 0.0;
                public double TempCelsiusRight = 0.0;
        }

        /** Update stats */
        default void updateStats(ShooterIOStats stats) {}

        /** Run motor at voltage */
        default void runVolts(double Volts) {}
    
        /** Stop Intake */
        default void stop() {}
    
        /** Run motor at velocity in rpm */
        default void runVelocity(double Rpm, double Feedforward) {}
    
        /** Config PID values for motor */
        default void setPID(double kP, double kI, double kD) {}
    
        /** Config FF values for motor */
        default void setFF(double kS, double kV, double kA) {}
    
        /** Run Characterization for motor at voltage */
        default void runCharacterization(double input) {}
       
}