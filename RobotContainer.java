// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm.ArmIOKraken;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveIO;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederKraken;
import frc.robot.subsystems.Feeder.FeederSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeKraken;
import frc.robot.subsystems.Intake.IntakeSim;
import frc.robot.subsystems.Shooter.ShooterIOKraken;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Extension.Extension;
import frc.robot.subsystems.Extension.ExtensionKraken;
import frc.robot.subsystems.Extension.ExtensionSim;



public class RobotContainer {
  public Drive drivetrain;
  public Intake intake;
  public Feeder feeder;
  private ArmSubsystem m_armSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  public Extension extension;
 

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        DriveKraken krakenDrive = new DriveKraken();
        IntakeKraken intakeKraken = new IntakeKraken();
        FeederKraken feederKraken = new FeederKraken();
        ExtensionKraken extensionKraken = new ExtensionKraken();
       

        this.drivetrain = new Drive(krakenDrive);
        this.intake = new Intake(intakeKraken);
        this.feeder = new Feeder(feederKraken);
        this.extension = new Extension(extensionKraken);
        

        m_armSubsystem = new ArmSubsystem((new ArmIOKraken()));
        m_shooterSubsystem = new ShooterSubsystem((new ShooterIOKraken()));
     
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        DriveSim simIO = new DriveSim();
        IntakeSim intakeSim = new IntakeSim();
        FeederSim feederSim = new FeederSim();
        ExtensionSim extensionSim = new ExtensionSim();

        this.drivetrain = new Drive(simIO);
        this.intake = new Intake(intakeSim);
        this.feeder = new Feeder(feederSim);
        this.extension = new Extension(extensionSim);
     
      }
    }

    configureBindings();
  }

  /* Driver Controller */
  private void configureBindings() {
 //   drivetrain.setDefaultCommand
   //   (drivetrain.run(
//        () -> 
 //         {
   //         drivetrain.teleopDrive(
     //         Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0, 
       //       Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0, 
         //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
         // }
    //  ));

 //   driver.rightTrigger().whileTrue(feeder.shoot()).whileFalse(feeder.idleCommand());

   // driver.x().whileTrue(intake.idleCommand());
   // driver.start().whileTrue(drivetrain.resetPidgeon());
    
    /* Operator Controller*/
  //  operator.b().whileTrue(Commands.parallel(m_armSubsystem.protectCommand(), m_shooterSubsystem.shootCommand()));
 //   operator.y().whileTrue(Commands.parallel(m_armSubsystem.batterCommand(), m_shooterSubsystem.shootCommand()));
   // operator.a().whileTrue(Commands.parallel(m_armSubsystem.zeroCommand(), m_shooterSubsystem.idleCommand()));
   // operator.leftBumper().whileTrue(Commands.parallel(m_armSubsystem.ampCommand(), m_shooterSubsystem.ampCommand()));
    // operator.x().whileTrue(m_shooterSubsystem.setPID());


    
    // driver.a().whileTrue((Commands.sequence(
    //   m_shooterSubsystem.shootCommand(), 
    //   m_armSubsystem.batterCommand(), 
    //   Commands.waitSeconds(2), 
    //   feeder.shoot(), 
    //   Commands.waitSeconds(.5),
    //   feeder.idleCommand(),
    //   m_armSubsystem.zeroCommand(),
    //   m_shooterSubsystem.idleCommand(),
    //   drivetrain.autoDrive(true, true),
    //   Commands.waitSeconds(1),
    //   drivetrain.autoDrive(true, false)))).whileFalse(drivetrain.autoDrive(false, false));
    operator.b().whileTrue(extension.extensionCommand()).whileFalse(extension.idleCommand());
 
    driver.leftTrigger().whileTrue(extension.extensionCommand()).whileFalse(extension.idleCommand());
    operator.leftTrigger().whileTrue(Commands.parallel(intake.intakeCommand(), feeder.FeederCommand())).whileFalse(Commands.parallel(intake.idleCommand(), feeder.idleCommand()));
    operator.rightTrigger().whileTrue(intake.ejectCommand()).whileFalse(intake.idleCommand());

  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.parallel(m_shooterSubsystem.shootCommand(), m_armSubsystem.batterCommand()), 
      Commands.waitSeconds(2), 
      feeder.shoot(), 
      Commands.waitSeconds(.5),
      feeder.idleCommand(),
      m_armSubsystem.zeroCommand(),
      m_shooterSubsystem.idleCommand(),
      // drivetrain.autoDrive(true, true),
      // Commands.waitSeconds(1),
      // drivetrain.autoDrive(true, false))
      extension.idleCommand(),
      extension.extensionCommand());
  }
}
