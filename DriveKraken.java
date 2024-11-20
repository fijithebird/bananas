package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveKraken extends SwerveDrivetrain implements DriveIO {


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private static final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric();

    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();



    public DriveKraken() {
        // drivetrain constants
        super(DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());

        for (int i = 0; i < ModuleCount; i++) {
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.SWERVE_DRIVE_GAINS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.SWERVE_STEER_GAINS, 1.0);

            // apply all types of current limits because 2024 ptsd :)
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_CURRENT_LIMITS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_CURRENT_LIMITS, 1.0);

            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_TORQUE_CONFIGS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_TORQUE_CONFIGS, 1.0);
        }
    }

    @Override
    public void percentDrive(double xPercent, double yPercent, double thetaPercent) {
        setControl(FIELD_CENTRIC
            .withVelocityX(xPercent * MAX_VELOCITY_METERS)
            .withVelocityY(yPercent * MAX_VELOCITY_METERS)
            .withRotationalRate(thetaPercent * MAX_ANGULAR_VELOCITY_RADS));
    }

    @Override
    public DriveIOdata update() {
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.pose = this.currentState.Pose;
            if (this.currentState.speeds != null) {
                this.iOdata.velocityX = this.currentState.speeds.vxMetersPerSecond;
                this.iOdata.velocityY = this.currentState.speeds.vyMetersPerSecond;
                this.iOdata.speed = Math.hypot(this.currentState.speeds.vxMetersPerSecond, this.currentState.speeds.vyMetersPerSecond);
            }
            
        }

        return this.iOdata;
    }

    public void setTeamRotation(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    @Override
    public void resetPidgeon() {
        m_pigeon2.setYaw(0);
    }
}
