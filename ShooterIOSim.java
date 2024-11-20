package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim shooterSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), ShooterConstants.kReduction, 0.00363458292);
    
    private final PIDController shooterController =
      new PIDController(ShooterConstants.gains.kP(), ShooterConstants.gains.kI(), ShooterConstants.gains.kD());

    private double AppliedVolts = 0.0;
    private Double SetpointRpm = null;
    private double Feedforward = 0.0;
   
    @Override
    public void updateStats(ShooterIOStats stats) {
        shooterSim.update(Constants.loopPeriodSecs);

        // control to setpoint
        //if (SetpointRpm != null && SetpointRpm != 0.0) {
        if (SetpointRpm != null) {    
            double av = shooterSim.getAngularVelocityRPM();
            double volt = shooterController.calculate(av, SetpointRpm)+ Feedforward;            
            runVolts(volt);
        }

        stats.PositionRadsLeft +=
            Units.radiansToRotations(shooterSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        stats.VelocityRpmLeft = shooterSim.getAngularVelocityRPM();
        stats.AppliedVoltsLeft = AppliedVolts;
        stats.SupplyCurrentAmpsLeft = shooterSim.getCurrentDrawAmps();  
    }



    @Override
    public void runVolts(double Volts) {
        SetpointRpm = null;
        //round to 2 decimal places 
        AppliedVolts = MathUtil.clamp(Volts, -12.0, 12.0);
        shooterSim.setInputVoltage(AppliedVolts);
    }

    @Override
    public void runVelocity(double Rpm, double Feedforward) {
        SetpointRpm = Rpm;
        this.Feedforward = Feedforward;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        shooterController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void runCharacterization(double input) {
        SetpointRpm = null;
        runVolts(input);
    }
}