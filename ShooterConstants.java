package frc.robot.subsystems.Shooter;

public class ShooterConstants {
    
    public static final int shooter1ID = 12;//right
    public static final int shooter2ID = 11;//left
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final Gains gains = new Gains(3.0, 0.0, 0.0, 13.35, 0.069, 0.0);

    
    
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {} 
}
