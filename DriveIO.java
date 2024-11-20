package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public interface DriveIO {
    // Data to be logged 
    class DriveIOdata {
        public double speed = 0.0;
        public double velocityX = 0.0;
        public double velocityY = 0.0;
        public Pose2d pose = new Pose2d(); 

        public double frontLeftDrivePositionRads = 0.0;
        public double frontLeftDriveVelocityRpm = 0.0;
        public double frontLeftDriveAppliedVolts = 0.0;
        public double frontLeftDriveSupplyCurrentAmps = 0.0;
        public double frontLeftDriveTorqueCurrentAmps = 0.0;
        public double frontLeftDriveTempCelsius = 0.0;
    
        public double frontRightDrivePositionRads = 0.0;
        public double frontRightDriveVelocityRpm = 0.0;
        public double frontRightDriveAppliedVolts = 0.0;
        public double frontRightDriveSupplyCurrentAmps = 0.0;
        public double frontRightDriveTorqueCurrentAmps = 0.0;
        public double frontRightDriveTempCelsius = 0.0;

        public double backLeftDrivePositionRads = 0.0;
        public double backLeftDriveVelocityRpm = 0.0;
        public double backLeftDriveAppliedVolts = 0.0;
        public double backLeftDriveSupplyCurrentAmps = 0.0;
        public double backLeftDriveTorqueCurrentAmps = 0.0;
        public double backLeftDriveTempCelsius = 0.0;
    
        public double backRightDrivePositionRads = 0.0;
        public double backRightDriveVelocityRpm = 0.0;
        public double backRightDriveAppliedVolts = 0.0;
        public double backRightDriveSupplyCurrentAmps = 0.0;
        public double backRightDriveTorqueCurrentAmps = 0.0;
        public double backRightDriveTempCelsius = 0.0;
    }

    /**
     * @param xPercent  Percentage of max speed in x direction
     * @param yPercent  Percentage of max speed in y direction
     * @param thetaPercent  Percentage of max holonomic rotation speed
     */
    default void percentDrive(double xPercent, double yPercent, double thetaPercent) {}

    default DriveIOdata update() {
        return null;
    }

    default void setTeamRotation(DriverStation.Alliance alliance) {}

    default void resetPidgeon() {}
}
