// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOStats;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOStats stats = new ShooterIOStats();
  private final ShuffleboardTab shooterShuffleboard;

  /* Shuffleboard entrys */
  private GenericEntry appliedVoltsLeft;
  private GenericEntry velocityRpmLeft;
  private GenericEntry positionLeft;
  private GenericEntry supplyCurrentLeft;
  private GenericEntry statorCurrentLeft;
  private GenericEntry tempLeft;

  private GenericEntry appliedVoltsRight;
  private GenericEntry velocityRpmRight;
  private GenericEntry positionRight;
  private GenericEntry supplyCurrentRight;
  private GenericEntry statorCurrentRight;
  private GenericEntry tempRight;

  private GenericEntry stateName;

  private GenericEntry shuffKP;
  private GenericEntry shuffKS;
  private GenericEntry shuffKV;

  
  private static final double _SHOOT = 3500.0; 
  private static final double _IDLE = 1000.0;
  private static final double _OFF = 0.0; 
  private static final double _AMP = 750.0; 

  public enum State {
    SHOOT(_SHOOT),
    IDLE(_IDLE),
    OFF(_OFF),
    AMP(_AMP);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  private State currentState = State.OFF;


  public ShooterSubsystem(ShooterIO io) {
    this.io = io; 

    this.shooterShuffleboard = Shuffleboard.getTab("Shooter");

    appliedVoltsLeft = this.shooterShuffleboard.add("Shooter Left Volts", 0.0).getEntry();
    velocityRpmLeft = this.shooterShuffleboard.add("Shooter RPM Left", 0.0).getEntry();
    positionLeft = this.shooterShuffleboard.add("Shooter Pos Left", 0.0).getEntry();
    supplyCurrentLeft = this.shooterShuffleboard.add("Shooter Supply Current Left", 0.0).getEntry();
    statorCurrentLeft = this.shooterShuffleboard.add("Shooter Stator Current Left", 0.0).getEntry();
    tempLeft = this.shooterShuffleboard.add("Shooter Temp Left", 0.0).getEntry();


    appliedVoltsRight = this.shooterShuffleboard.add("Shooter Right Volts", 0.0).getEntry();
    velocityRpmRight = this.shooterShuffleboard.add("Shooter RPM Right", 0.0).getEntry();
    positionRight = this.shooterShuffleboard.add("Shooter Pos Right", 0.0).getEntry();
    supplyCurrentRight = this.shooterShuffleboard.add("Shooter Supply Current Right", 0.0).getEntry();
    statorCurrentRight = this.shooterShuffleboard.add("Shooter Stator Current Right", 0.0).getEntry();
    tempRight = this.shooterShuffleboard.add("Shooter Temp Right", 0.0).getEntry();


    stateName = this.shooterShuffleboard.add("Shooter State", this.currentState.name()).getEntry();

    shuffKP = this.shooterShuffleboard.add("Shooter KP", 0.0).getEntry();
    shuffKS = this.shooterShuffleboard.add("Shooter KS", 0.0).getEntry();
    shuffKV = this.shooterShuffleboard.add("Shooter KV", 0.0).getEntry();

  }

  public Command shootCommand() {
    
    return runOnce(() -> {currentState = State.SHOOT;});
  }

  public Command idleCommand() {
  
    return runOnce(() -> {currentState = State.IDLE;});
  }

    public Command OFFCommand() {
  
    return runOnce(() -> {currentState = State.OFF;});
  }
    public Command ampCommand() {
  
    return runOnce(() -> {currentState = State.AMP;});
  }

  public boolean exampleCondition() {

    return false;
  }

  @Override
  public void periodic() {

    double motorRPM = currentState.rpm;
    if(currentState == State.IDLE || currentState == State.SHOOT || currentState == State.AMP){
    io.runVelocity(motorRPM, 0.0); 
    }
    else{
      
    }

    io.updateStats(stats);
    
    UpdateTelemetry();
  
   
  }

  private void UpdateTelemetry() {
    SmartDashboard.putNumber("ShooterGoal", currentState.rpm);
    appliedVoltsLeft.setDouble(stats.AppliedVoltsLeft);
    velocityRpmLeft.setDouble(stats.VelocityRpmLeft);
    positionLeft.setDouble(stats.PositionRadsLeft);
    supplyCurrentLeft.setDouble(stats.SupplyCurrentAmpsLeft);
    statorCurrentLeft.setDouble(stats.TorqueCurrentAmpsLeft);
    tempLeft.setDouble(stats.TempCelsiusLeft);

    appliedVoltsRight.setDouble(stats.AppliedVoltsRight);
    velocityRpmRight.setDouble(stats.VelocityRpmRight);
    positionRight.setDouble(stats.PositionRadsRight);
    supplyCurrentRight.setDouble(stats.SupplyCurrentAmpsRight);
    statorCurrentRight.setDouble(stats.TorqueCurrentAmpsRight);
    tempRight.setDouble(stats.TempCelsiusRight);

    stateName.setString(currentState.name());
  }

  public Command setPID() {
    return runOnce(() -> {io.setPID(
      this.shuffKP.getDouble(0.0),
      this.shuffKS.getDouble(0.0), 
      this.shuffKV.getDouble(0.0));});
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}