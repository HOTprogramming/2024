package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public final class Constants {

    public final class ShooterConstants {
        public static final int RIGHT_FLYWHEEL_CAN = 12;
        public static final int LEFT_FLYWHEEL_CAN = 11;
        public static final int FEEDER_CAN = 13;

        public static final double TARGET_SPEED_INCREMENT = 5;
        public static final double FEEDER_SPEED = 0.4;

        public static final double FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
        public static final double FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed

        public static final double SHOOTER_KP = 0.25;
        public static final double SHOOTER_KI = 0.5;
        public static final double SHOOTER_KD = 0.0001;
        public static final double LEFT_SHOOTER_KV = 0.133;
        public static final double RIGHT_SHOOTER_KV = 0.138;
        public static final double LEFT_SHOOTER_KS = 0.384;
        public static final double RIGHT_SHOOTER_KS = 0.38;
        public static final double SHOOTER_PEAK_VOLTAGE = 12;
    }


    public final class Auton {
        public static final double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
        public static final double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;
    }
  
  
    public final class Camera {
        public static final String FRONT_CAMERA_NAME = "front_camera";
        
        public static final Translation3d FRONT_CAMERA_RELATIVE_POSITION = new Translation3d(.35, .29, .165);
        public static final Rotation3d FRONT_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-8), 0);
        public static final Transform3d FRONT_CAMERA_TRANSFROM = new Transform3d(FRONT_CAMERA_RELATIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION);

        public static final Translation3d REAR_CAMERA_RELATIVE_POSITION = new Translation3d(0, -.3, .1);
        public static final Rotation3d REAR_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-8), Units.degreesToRadians(180));
        public static final Transform3d REAR_CAMERA_TRANSFROM = new Transform3d(REAR_CAMERA_RELATIVE_POSITION, REAR_CAMERA_RELATIVE_ROTATION);



        public static final String REAR_CAMERA_NAME = "back_camera";
    }
  
    public final class Drivetrain {
    public static final double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
    public static final double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;

    public static final double ROBOT_LENGTH_INCHES = 20.25;
    public static final double ROBOT_WITDTH_INCHES = 20.25;
    public static final double MAX_VELOCITY_METERS = 6.37032; // from SDS
    // public static final double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
    // public static final double MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
    public static final double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / 0.7274007458;

    // WCS Docs X3 11 https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options 
    // SWERVE BUILDER
    private static final Slot0Configs SWERVE_STEER_GAINS = new Slot0Configs()
    .withKP(100).withKI(0).withKD(0.2)
    .withKS(0).withKV(1.5).withKA(0);

    private static final Slot0Configs SWERVE_DRIVE_GAINS = new Slot0Configs()
    .withKP(3).withKI(0).withKD(0)
    .withKS(0).withKV(0).withKA(0);
    
    // private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

    // private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

    
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;



    private static final double WHEEL_SLIP_CURRENT = 300.0; // *tune later

    // Meters per second theroretical max speed at 12 volts
    public static final double FREE_SPEED_12V = 6.37032;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716; // tune later

    private static final double kDriveGearRatio = 5.01;
    private static final double kSteerGearRatio = 13.3714;
    private static final double kWheelRadiusInches = 2;

    // Are steer motors GENERALLY reversed
    private static final boolean kSteerMotorReversed = true;


    

    private static final String CANBUS_NAME = "drivetrain";
    private static final int PIDGEON_CAN = 50;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    // REAL swerve rotational inertia 3.05 lbs*in^2 or 0.0008925509 kg * m^2
    private static final double kDriveInertia = 0.001;

    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;   


    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
    .withPigeon2Id(PIDGEON_CAN)
    .withCANbusName(CANBUS_NAME);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
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
        private static final boolean SWERVE_FRONT_LEFT_DRIVE_UNINVERT = !true;
        private static final boolean SWERVE_FRONT_LEFT_STEER_UNINVERT = false;
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 43;
        private static final double kFrontLeftEncoderOffset = 0.440673828125 * Math.PI;

        private static final double kFrontLeftXPosInches = 10.125;
        private static final double kFrontLeftYPosInches = 10.125;

        // Front Right
        private static final boolean SWERVE_FRONT_RIGHT_DRIVE_UNINVERT = !true;
        private static final boolean SWERVE_FRONT_RIGHT_STEER_UNINVERT = true;
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 4;
        private static final int kFrontRightEncoderId = 41;
        private static final double kFrontRightEncoderOffset = 0.098876953125 * Math.PI;

        private static final double kFrontRightXPosInches = 10.125;
        private static final double kFrontRightYPosInches = -10.125;

        // Back Left
        private static final boolean SWERVE_BACK_LEFT_DRIVE_UNINVERT = !false;
        private static final boolean SWERVE_BACK_LEFT_STEER_UNINVERT = false;
        private static final int kBackLeftDriveMotorId = 5;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 42;
        private static final double kBackLeftEncoderOffset = -0.450439453125 * Math.PI;

        private static final double kBackLeftXPosInches = -10.125;
        private static final double kBackLeftYPosInches = 10.125;


        // Back Right
        private static final boolean SWERVE_BACK_RIGHT_DRIVE_UNINVERT = !true;
        private static final boolean SWERVE_BACK_RIGHT_STEER_UNINVERT = false;
        private static final int kBackRightDriveMotorId = 7;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 40;
        private static final double kBackRightEncoderOffset = -0.44140625 * Math.PI;

        private static final double kBackRightXPosInches = -10.125;
        private static final double kBackRightYPosInches = -10.125;


        public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset / Math.PI, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), !SWERVE_FRONT_LEFT_DRIVE_UNINVERT)
        .withSteerMotorInverted(!SWERVE_FRONT_LEFT_STEER_UNINVERT);
        public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset / Math.PI, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), !SWERVE_FRONT_RIGHT_DRIVE_UNINVERT)
        .withSteerMotorInverted(!SWERVE_FRONT_RIGHT_STEER_UNINVERT);
        public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset / Math.PI, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), !SWERVE_BACK_LEFT_DRIVE_UNINVERT)
        .withSteerMotorInverted(!SWERVE_BACK_LEFT_STEER_UNINVERT);
        public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset / Math.PI, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), !SWERVE_BACK_RIGHT_DRIVE_UNINVERT)
        .withSteerMotorInverted(!SWERVE_BACK_RIGHT_STEER_UNINVERT);
    }
    

    public final class ArmConstants {
        public static final int CANCODER_CAN = 44;
        public static final int ARM_CAN = 9;
        public static final double CRUISEVELOCITY = 400;
        public static final double ACCELERATION = 400;
        public static final double JERK = 2000;
        public static final double ARMKP = 380;
        public static final double ARMKI = 0;
        public static final double ARMKD = 0;
        public static final double ARMKV = 0.8;
        public static final double ARMKS = 0.1;
        public static final double ZERO = 95;
        public static final double SHOOT = 118;
        
    }
}