package frc.robot.subsystems.Arm;

public class ArmConstants {
    
    public static final int arm1MotorID = 32;//right
    public static final int arm2MotorID = 31;//left
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains armGains = new MMGains(200, 100, 200, 250, 0.0, 0.0, 2.0, 0.25);

    
    
    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}
