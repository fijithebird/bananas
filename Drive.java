package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;

public class Drive extends SubsystemBase {
    private DriveIO driveIO;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;

    private boolean autoDriving;
    private boolean autoDriveDirection; // true is blue


    public Drive(DriveIO driveIO) { 
        this.driveIO = driveIO;
        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed", 0).getEntry();
        poseEntry = driveTab.add("Pose", 0).getEntry();

    }

    public void teleopDrive(double driveX, double driveY, double driveTheta)  {
        driveIO.percentDrive(
            driveX <= 0 ? -(driveX * driveX) : (driveX * driveX),
            driveY <= 0 ? -(driveY * driveY) : (driveY * driveY),
            driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta));
    }

    /**
     * 
     * @param direction true is blue
     * @return
     */
    public Command autoDrive(boolean direction, boolean on) {
        return runOnce(() -> {this.autoDriving = on;
        this.autoDriveDirection = direction;});
    }

    @Override
    public void periodic() {
        this.iOdata = driveIO.update();
        speedEntry.setDouble(this.iOdata.speed);
        if (this.iOdata.pose != null) {
            poseEntry.setDoubleArray(new Double[]{
                this.iOdata.pose.getX(), 
                this.iOdata.pose.getY(), 
                this.iOdata.pose.getRotation().getDegrees()});
        } 
        
        if (this.autoDriving) {
            teleopDrive(this.autoDriveDirection ? -0.66 : 0.33, 0, 0);
        }
    }

    public Command resetPidgeon() {
        return runOnce(() -> {driveIO.resetPidgeon();});
    }

    public void init() {
    }

    public void setTeamRotation() {
    }
}
