// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOStats;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOStats stats = new ArmIOStats();
  private final ShuffleboardTab armShuffleboard;

  /* Shuffleboard entrys */
  private GenericEntry velocityRpmLeader;
  private GenericEntry positionLeader;
  private GenericEntry supplyCurrentLeader;
  private GenericEntry statorCurrentLeader;
  private GenericEntry tempLeader;

  private GenericEntry velocityRpmCancoder;
  private GenericEntry positionCancoder;

  private GenericEntry velocityRpmFollower;
  private GenericEntry positionFollower;
  private GenericEntry supplyCurrentFollower;
  private GenericEntry statorCurrentFollower;
  private GenericEntry tempFollower;

  private GenericEntry goalPose;
  private GenericEntry stateName;

  private GenericEntry f_fusedSensorOutOfSync;
  private GenericEntry sf_fusedSensorOutOfSync;
  private GenericEntry f_remoteSensorInvalid;
  private GenericEntry sf_remoteSensorInvalid;
  
  //lowest value is 0.984, highest value is 1.17
  private static final double ZERO_POS = 1.0;
  private static final double BATTER_POS = 1.05;
  private static final double PROTECTED_POS = 1.00045;
  private static final double AMP_POS = 1.181;

  public enum State {
    ZERO(ZERO_POS),
    BATTER(BATTER_POS),
    PROTECTED(PROTECTED_POS),
    AMP(AMP_POS);

    private final double armPos;
  
    private State (double armPosIn) {
        this.armPos = armPosIn; 
    }

  }

  private State currentState = State.ZERO;

  public ArmSubsystem(ArmIO io) {
    this.io = io; 

    this.armShuffleboard = Shuffleboard.getTab("Arm");

    velocityRpmLeader = this.armShuffleboard.add("Arm RPM Leader", 0.0).getEntry();
    positionLeader = this.armShuffleboard.add("Arm Pos Leader", 0.0).getEntry();
    supplyCurrentLeader = this.armShuffleboard.add("Arm Supply Current Leader", 0.0).getEntry();
    statorCurrentLeader = this.armShuffleboard.add("Arm Stator Current Leader", 0.0).getEntry();
    tempLeader = this.armShuffleboard.add("Arm Temp Leader", 0.0).getEntry();

    velocityRpmCancoder = this.armShuffleboard.add("Arm RPM Cancoder", 0.0).getEntry();
    positionCancoder = this.armShuffleboard.add("Arm Pos Cancoder", 0.0).getEntry();

    velocityRpmFollower = this.armShuffleboard.add("Arm RPM Follower", 0.0).getEntry();
    positionFollower = this.armShuffleboard.add("Arm Pos Follower", 0.0).getEntry();
    supplyCurrentFollower = this.armShuffleboard.add("Arm Supply Current Follower", 0.0).getEntry();
    statorCurrentFollower = this.armShuffleboard.add("Arm Stator Current Follower", 0.0).getEntry();
    tempFollower = this.armShuffleboard.add("Arm Temp Follower", 0.0).getEntry();

    goalPose = this.armShuffleboard.add("Arm Goal", 0.0).getEntry();
    stateName = this.armShuffleboard.add("Arm State", this.currentState.name()).getEntry();

    f_fusedSensorOutOfSync = this.armShuffleboard.add("f_fusedSensorOutOfSync", false).getEntry();
    sf_fusedSensorOutOfSync = this.armShuffleboard.add("sf_fusedSensorOutOfSync", false).getEntry();
    f_remoteSensorInvalid = this.armShuffleboard.add("f_remoteSensorInvalid", false).getEntry();
    sf_remoteSensorInvalid = this.armShuffleboard.add("sf_remoteSensorInvalid", false).getEntry();
  }



  public Command protectCommand() {
  return runOnce(() -> {currentState = State.PROTECTED; });
  }

  public Command zeroCommand() {
   return runOnce(() -> {currentState = State.ZERO;});
  }

  public Command batterCommand() {
    return runOnce(() -> {currentState = State.BATTER;});
  }

    public Command ampCommand() {
    return runOnce(() -> {currentState = State.AMP;});
  }

  public boolean armExampleCondition() {
    return false;
  }

  @Override
  public void periodic() {

    double armControl = currentState.armPos;
    io.setArmMotorControl(armControl);
    io.updateArmStats(stats);
    
    UpdateTelemetry();
  
   
  }

  private void UpdateTelemetry() {
    velocityRpmLeader.setDouble(stats.armVelocity);
    positionLeader.setDouble(stats.armPosition);
    supplyCurrentLeader.setDouble(stats.SupplyCurrentAmpsLeader);
    statorCurrentLeader.setDouble(stats.TorqueCurrentAmpsLeader);
    tempLeader.setDouble(stats.TempCelsiusLeader);

    velocityRpmCancoder.setDouble(stats.cancoderPosition);
    positionCancoder.setDouble(stats.cancoderVelocity);

    velocityRpmFollower.setDouble(stats.armFollowerVelocity);
    positionFollower.setDouble(stats.armFollowerPosition);
    supplyCurrentFollower.setDouble(stats.SupplyCurrentAmpsFollower);
    statorCurrentFollower.setDouble(stats.TorqueCurrentAmpsFollower);
    tempFollower.setDouble(stats.TempCelsiusFollower);

    goalPose.setDouble(currentState.armPos);
    stateName.setString(currentState.name());

    f_fusedSensorOutOfSync.setBoolean(stats.f_fusedSensorOutOfSync);
    sf_fusedSensorOutOfSync.setBoolean(stats.sf_fusedSensorOutOfSync);
    f_remoteSensorInvalid.setBoolean(stats.f_remoteSensorInvalid);
    sf_remoteSensorInvalid.setBoolean(stats.sf_remoteSensorInvalid);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}