package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
public class FeederSim implements FeederIO {
    private final FlywheelSim intakeSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.00363458292);
    
    private final PIDController intakeController =
      new PIDController(FeederConstants.FeederMotorConfig.Slot0.kP, FeederConstants.FeederMotorConfig.Slot0.kI, FeederConstants.FeederMotorConfig.Slot0.kD);

    private double AppliedVolts = 0.0;
    private Double SetpointRpm = null;
    private double Feedforward = 0.0;
   
    @Override
    public void updateStats(FeederIOStats stats) {
        intakeSim.update(Constants.loopPeriodSecs);

        // control to setpoint
        //if (SetpointRpm != null && SetpointRpm != 0.0) {
        if (SetpointRpm != null) {    
            double av = intakeSim.getAngularVelocityRPM();
            double volt = intakeController.calculate(av, SetpointRpm)+ Feedforward;            
            runVolts(volt);
        }

        stats.PositionRads +=
            Units.radiansToRotations(intakeSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        stats.VelocityRpm = intakeSim.getAngularVelocityRPM();
        stats.AppliedVolts = AppliedVolts;
        stats.SupplyCurrentAmps = intakeSim.getCurrentDrawAmps();  
    }



    @Override
    public void runVolts(double Volts) {
        SetpointRpm = null;
        //round to 2 decimal places 
        AppliedVolts = MathUtil.clamp(Volts, -12.0, 12.0);
        intakeSim.setInputVoltage(AppliedVolts);
    }

    @Override
    public void intake(double Rpm) {
        SetpointRpm = Rpm;
        this.Feedforward = Feedforward;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        intakeController.setPID(kP, kI, kD);
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
