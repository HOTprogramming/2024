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

public class CamBotConstants extends ConstantsBase {

    public class Auton extends ConstantsBase.Auton {
        public static final double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
        public static final double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;
        
    }
  
  
    public class Camera extends ConstantsBase.Camera {
        public static final boolean HAS_CAMERA = true;
        
        public static final String FRONT_CAMERA_NAME = "front_camera";
        
        public static final Translation3d FRONT_CAMERA_REALITIVE_POSITION = new Translation3d(.35, .29, .165);
        public static final Rotation3d FRONT_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-8), 0);
        public static final Transform3d FRONT_CAMERA_TRANSFROM = new Transform3d(FRONT_CAMERA_REALITIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION);

        public static final Translation3d REAR_CAMERA_REALITIVE_POSITION = new Translation3d(0, -.3, .1);
        public static final Rotation3d REAR_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180));
        public static final Transform3d REAR_CAMERA_TRANSFROM = new Transform3d(REAR_CAMERA_REALITIVE_POSITION, REAR_CAMERA_RELATIVE_ROTATION);



        public static final String REAR_CAMERA_NAME = "back_camera";
    }

    public class Drivetrain extends ConstantsBase.Drivetrain {
        public static final double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
        public static final double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;

        public static final double PIDGEON_OFFSET_DEGREES = 90;

        public static final double ROBOT_LENGTH_INCHES = 20.25;
        public static final double ROBOT_WITDTH_INCHES = 20.25;
        public static final double MAX_VELOCITY_METERS = 6.37032; // from SDS
        // public static final double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
        // public static final double MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
        public static final double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / 0.7274007458;

        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.05)
            .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        private static final double kSpeedAt12VoltsMps = 6.0;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 12.8;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = false;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "drivetrain";
        private static final int kPigeonId = 50;


        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 4;
        private static final int kFrontLeftEncoderId = 14;
        private static final double kFrontLeftEncoderOffset = 0.385986328125;

        private static final double kFrontLeftXPosInches = 8.8;
        private static final double kFrontLeftYPosInches = 8.8;

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 3;
        private static final int kFrontRightEncoderId = 13;
        private static final double kFrontRightEncoderOffset = -0.30908203125;

        private static final double kFrontRightXPosInches = 8.8;
        private static final double kFrontRightYPosInches = -8.8;

        // Back Left
        private static final int kBackLeftDriveMotorId = 6;
        private static final int kBackLeftSteerMotorId = 8;
        private static final int kBackLeftEncoderId = 18;
        private static final double kBackLeftEncoderOffset = -0.33203125;

        private static final double kBackLeftXPosInches = -8.8;
        private static final double kBackLeftYPosInches = 8.8;

        // Back Right
        private static final int kBackRightDriveMotorId = 5;
        private static final int kBackRightSteerMotorId = 7;
        private static final int kBackRightEncoderId = 17;
        private static final double kBackRightEncoderOffset = -0.412353515625;

        private static final double kBackRightXPosInches = -8.8;
        private static final double kBackRightYPosInches = -8.8;


        public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

                
    }
}
