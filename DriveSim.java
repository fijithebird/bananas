package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain.DriveConstants.DriveConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;

public class DriveSim extends SwerveDrivetrain  implements DriveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private static final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric();

    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();


    
    public DriveSim() {
        super(DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());
        startSimThread();
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
        updateSimState(.02, 12);
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

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
