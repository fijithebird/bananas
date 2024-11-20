// package frc.robot.subsystems.Arm;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
// import frc.robot.Constants;

// public class ArmIOSim implements ArmIO {
//     private final FlywheelSim armSim =
//       new FlywheelSim(DCMotor.getKrakenX60Foc(1), ArmConstants.kReduction, 0.00363458292);
    
//     private final PIDController armController =
//       new PIDController(ArmConstants.gains.kP(), ArmConstants.gains.kI(), ArmConstants.gains.kD());

//     private final TrapezoidProfileSubsystem armController = new TrapezoidProfile(50,5);

//     private double AppliedVolts = 0.0;
//     private Double SetpointRpm = null;
//     private double Feedforward = 0.0;
   
//     @Override
//     public void updateArmStats(ArmIOStats stats) {
//         armSim.update(Constants.loopPeriodSecs);

//         // control to setpoint
//         //if (SetpointRpm != null && SetpointRpm != 0.0) {
//         if (SetpointRpm != null) {    
//             double av = armSim.getAngularVelocityRPM();
//             double volt = armController.calculate(av, SetpointRpm)+ Feedforward;            
//             runVolts(volt);
//         }

//         stats.PositionRads +=
//             Units.radiansToRotations(armSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
//         stats.VelocityRpm = armSim.getAngularVelocityRPM();
//         stats.AppliedVolts = AppliedVolts;
//         stats.SupplyCurrentAmps = armSim.getCurrentDrawAmps();  
//     }



//     @Override
//     public void runVolts(double Volts) {
//         SetpointRpm = null;
//         //round to 2 decimal places 
//         AppliedVolts = MathUtil.clamp(Volts, -12.0, 12.0);
//         armSim.setInputVoltage(AppliedVolts);
//     }

//     @Override
//     public void runVelocity(double Rpm, double Feedforward) {
//         SetpointRpm = Rpm;
//         this.Feedforward = Feedforward;
//     }

//     @Override
//     public void setPID(double kP, double kI, double kD) {
//         armController.setPID(kP, kI, kD);
//     }

//     @Override
//     public void stop() {
//         runVolts(0.0);
//     }

//     @Override
//     public void runCharacterization(double input) {
//         SetpointRpm = null;
//         runVolts(input);
//     }
// }