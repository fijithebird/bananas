package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import java.lang.Math;


import frc.robot.Constants;


public class DriveConstants {
    static CurrentLimitsConfigs DRIVE_CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(90)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentThreshold(90)
        .withSupplyTimeThreshold(0.0);

    static CurrentLimitsConfigs AZIMUTH_CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentThreshold(40)
        .withSupplyTimeThreshold(0.0);

    static TorqueCurrentConfigs DRIVE_TORQUE_CONFIGS = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(65)
        .withPeakReverseTorqueCurrent(-65);

    static TorqueCurrentConfigs AZIMUTH_TORQUE_CONFIGS = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(50)
        .withPeakReverseTorqueCurrent(-50);

    static double ROBOT_LENGTH_INCHES = 20.25;
    static double ROBOT_WITDTH_INCHES = 20.25;
    static double MAX_VELOCITY_METERS = 7.37032; // from SDS
    // public MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
    // public MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
    static double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / (0.7274007458 * .8);

    // WCS Docs X3 11 https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options 
    // SWERVE BUILDER
    static Slot0Configs SWERVE_STEER_GAINS = new Slot0Configs()
    .withKP(100).withKI(0).withKD(.2) // 400, 0, 8 , 0 ,1.5, 0
    .withKS(0).withKV(1.5).withKA(0);

    static Slot0Configs SWERVE_DRIVE_GAINS = new Slot0Configs()
    .withKP(3).withKI(0).withKD(0)
    .withKS(0).withKV(0).withKA(0);
    

    static double WHEEL_SLIP_CURRENT = 650.0; // *tune later

    // Meters per second theroretical max speed at 12 volts
    static double FREE_SPEED_12V = 7.37032;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    static double kCoupleRatio = 3.5714285714285716; // tune later

    static double kDriveGearRatio = 5.01;
    static double kSteerGearRatio = 13.3714;
    static double kWheelRadiusInches = 2;

    // Are steer motors GENERALLY reversed
    static boolean kSteerMotorReversed = true;


    static String CANBUS_NAME = "drivetrain";
    static int PIDGEON_CAN = 50;

    // These are only used for simulation
    static double kSteerInertia = 0.00001;
    // REAL swerve rotational inertia 3.05 lbs*in^2 or 0.0008925509 kg * m^2
    static double kDriveInertia = 0.001;

    // Simulated voltage necessary to overcome friction
    static double kSteerFrictionVoltage = 0.25;
    static double kDriveFrictionVoltage = 0.25; 
    
    static ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
    static ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;


    static SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
        .withPigeon2Id(PIDGEON_CAN)
        .withCANbusName(CANBUS_NAME);

    static SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kSteerGearRatio)
        .withWheelRadius(kWheelRadiusInches)
        .withSlipCurrent(WHEEL_SLIP_CURRENT)
        .withSteerMotorGains(SWERVE_STEER_GAINS)
        .withDriveMotorGains(SWERVE_DRIVE_GAINS)
        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPE)
        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPE)
        .withSpeedAt12VoltsMps(FREE_SPEED_12V)
        .withSteerInertia(kSteerInertia)
        .withDriveInertia(kDriveInertia)
        .withSteerFrictionVoltage(kSteerFrictionVoltage)
        .withDriveFrictionVoltage(kDriveFrictionVoltage)
        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
        .withCouplingGearRatio(kCoupleRatio)
        .withSteerMotorInverted(kSteerMotorReversed);


                
    
    // offsets in radians

    // Front Left
    static boolean SWERVE_FRONT_LEFT_DRIVE_UNINVERT = !true;
    static boolean SWERVE_FRONT_LEFT_STEER_UNINVERT = false;
    static int kFrontLeftDriveMotorId = 1;
    static int kFrontLeftSteerMotorId = 2;
    static int kFrontLeftEncoderId = 43;
    static double kFrontLeftEncoderOffset = 0.441162109375 * Math.PI;

    static double kFrontLeftXPosInches = 10.125;
    static double kFrontLeftYPosInches = 10.125;

    // Front Right
    static boolean SWERVE_FRONT_RIGHT_DRIVE_UNINVERT = !true;
    static boolean SWERVE_FRONT_RIGHT_STEER_UNINVERT = true;
    static int kFrontRightDriveMotorId = 3;
    static int kFrontRightSteerMotorId = 4;
    static int kFrontRightEncoderId = 41;
    static double kFrontRightEncoderOffset = 0.12353515625 * Math.PI;

    static double kFrontRightXPosInches = 10.125;
    static double kFrontRightYPosInches = -10.125;

    // Back Left
    static boolean SWERVE_BACK_LEFT_DRIVE_UNINVERT = !false;
    static boolean SWERVE_BACK_LEFT_STEER_UNINVERT = false;
    static int kBackLeftDriveMotorId = 5;
    static int kBackLeftSteerMotorId = 6;
    static int kBackLeftEncoderId = 42;
    static double kBackLeftEncoderOffset = -0.452880859375 * Math.PI;

    static double kBackLeftXPosInches = -10.125;
    static double kBackLeftYPosInches = 10.125;


    // Back Right
    static boolean SWERVE_BACK_RIGHT_DRIVE_UNINVERT = !true;
    static boolean SWERVE_BACK_RIGHT_STEER_UNINVERT = false;
    static int kBackRightDriveMotorId = 7;
    static int kBackRightSteerMotorId = 8;
    static int kBackRightEncoderId = 40;
    static double kBackRightEncoderOffset = -0.431884765625 * Math.PI;

    static double kBackRightXPosInches = -10.125;
    static double kBackRightYPosInches = -10.125;

    static SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
    kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset / Math.PI, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), !SWERVE_FRONT_LEFT_DRIVE_UNINVERT)
    .withSteerMotorInverted(!SWERVE_FRONT_LEFT_STEER_UNINVERT);
    static SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset / Math.PI, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), !SWERVE_FRONT_RIGHT_DRIVE_UNINVERT)
    .withSteerMotorInverted(!SWERVE_FRONT_RIGHT_STEER_UNINVERT);
    static SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset / Math.PI, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), !SWERVE_BACK_LEFT_DRIVE_UNINVERT)
    .withSteerMotorInverted(!SWERVE_BACK_LEFT_STEER_UNINVERT);
    static SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset / Math.PI, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), !SWERVE_BACK_RIGHT_DRIVE_UNINVERT)
    .withSteerMotorInverted(!SWERVE_BACK_RIGHT_STEER_UNINVERT);

    public static final DriveConfig DriveConfig =
        switch (Constants.getRobot()) {
            case COMPBOT -> new DriveConfig(
                FRONT_LEFT_MODULE_CONSTANTS, 
                FRONT_RIGHT_MODULE_CONSTANTS, 
                BACK_LEFT_MODULE_CONSTANTS, 
                BACK_RIGHT_MODULE_CONSTANTS,
                DRIVETRAIN_CONSTANTS,
                MAX_VELOCITY_METERS,
                MAX_ANGULAR_VELOCITY_RADS,
                DRIVE_CURRENT_LIMITS,
                AZIMUTH_CURRENT_LIMITS
                );
            case DEVBOT -> new DriveConfig(
                FRONT_LEFT_MODULE_CONSTANTS, 
                FRONT_RIGHT_MODULE_CONSTANTS, 
                BACK_LEFT_MODULE_CONSTANTS, 
                BACK_RIGHT_MODULE_CONSTANTS,
                DRIVETRAIN_CONSTANTS,
                MAX_VELOCITY_METERS,
                MAX_ANGULAR_VELOCITY_RADS,
                DRIVE_CURRENT_LIMITS,
                AZIMUTH_CURRENT_LIMITS);
            case SIMBOT -> new DriveConfig(
                FRONT_LEFT_MODULE_CONSTANTS, 
                FRONT_RIGHT_MODULE_CONSTANTS, 
                BACK_LEFT_MODULE_CONSTANTS, 
                BACK_RIGHT_MODULE_CONSTANTS,
                DRIVETRAIN_CONSTANTS,
                MAX_VELOCITY_METERS,
                MAX_ANGULAR_VELOCITY_RADS,
                DRIVE_CURRENT_LIMITS,
                AZIMUTH_CURRENT_LIMITS);
        };


    public record DriveConfig(
        SwerveModuleConstants FRONT_LEFT, 
        SwerveModuleConstants FRONT_RIGHT, 
        SwerveModuleConstants BACK_LEFT, 
        SwerveModuleConstants BACK_RIGHT,
        SwerveDrivetrainConstants DRIVETRAIN,
        double MAX_VELOCITY,
        double MAX_ANGULAR_VELOCITY,
        CurrentLimitsConfigs DRIVE_CURRENT,
        CurrentLimitsConfigs AZIMUTH_CURRENT) {}

}