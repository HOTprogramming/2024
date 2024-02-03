package frc.robot.Constants;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class CamBotConstants extends ConstantsBase {

    public class Auton extends ConstantsBase.Auton {
        public Auton() {
            AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
            AUTON_DEFAULT_MAX_ACCEL_METERS = 2;
        }
    }
  
  
    public class Camera extends ConstantsBase.Camera {
        public Camera() {
            HAS_CAMERA = true;
               
            FRONT_CAMERA_NAME = "front_camera";
            
            FRONT_CAMERA_REALITIVE_POSITION = new Translation3d(0.27, -0.2, 0.175);
            FRONT_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-8), 0);
            FRONT_CAMERA_TRANSFROM = new Transform3d(FRONT_CAMERA_REALITIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION);
            
            REAR_CAMERA_REALITIVE_POSITION = new Translation3d(0, -0.28, 0.14);
            REAR_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180));
            REAR_CAMERA_TRANSFROM = new Transform3d(REAR_CAMERA_REALITIVE_POSITION, REAR_CAMERA_RELATIVE_ROTATION);



            REAR_CAMERA_NAME = "back_camera";
        }
    }

    public class Drivetrain extends ConstantsBase.Drivetrain {
        public Drivetrain() {
            
            AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
            AUTON_DEFAULT_MAX_ACCEL_METERS = 2;

            ROBOT_LENGTH_INCHES = 20.25;
            ROBOT_WITDTH_INCHES = 20.25;
            MAX_VELOCITY_METERS = 6.37032; // from SDS
            // public MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
            // public MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
            MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / 0.7274007458;

            // Both sets of gains need to be tuned to your individual robot.

            // The steer motor uses any SwerveModule.SteerRequestType control request with the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            SWERVE_STEER_GAINS = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.05)
                .withKS(0).withKV(1.5).withKA(0);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            SWERVE_DRIVE_GAINS = new Slot0Configs()
                .withKP(3).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

            // The closed-loop output type to use for the steer motors;
            // This affects the PID/FF gains for the steer motors
            STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;
            // The closed-loop output type to use for the drive motors;
            // This affects the PID/FF gains for the drive motors
            DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

            // The stator current at which the wheels start to slip;
            // This needs to be tuned to your individual robot
            WHEEL_SLIP_CURRENT = 300.0;

            // Theoretical free speed (m/s) at 12v applied output;
            // This needs to be tuned to your individual robot
            FREE_SPEED_12V = 6.0;

            // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
            // This may need to be tuned to your individual robot
            kCoupleRatio = 3.5714285714285716;

            kDriveGearRatio = 6.746031746031747;
            kSteerGearRatio = 12.8;
            kWheelRadiusInches = 2;

            kSteerMotorReversed = false;
            boolean kInvertLeftSide = false;
            boolean kInvertRightSide = true;

            CANBUS_NAME = "drivetrain";
            PIDGEON_CAN = 50;


            // These are only used for simulation
            kSteerInertia = 0.00001;
            kDriveInertia = 0.001;

            DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                    .withPigeon2Id(PIDGEON_CAN)
                    .withCANbusName(CANBUS_NAME);

            ConstantCreator = new SwerveModuleConstantsFactory()
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
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withSteerMotorInverted(kSteerMotorReversed);


            // Front Left
            kFrontLeftDriveMotorId = 6;
            kFrontLeftSteerMotorId = 8;
            kFrontLeftEncoderId = 18;
            kFrontLeftEncoderOffset = 0.41455078125;

            kFrontLeftXPosInches = 8.8;
            kFrontLeftYPosInches = 8.8;

            // Front Right
            kFrontRightDriveMotorId = 2;
            kFrontRightSteerMotorId = 4;
            kFrontRightEncoderId = 14;
            kFrontRightEncoderOffset = -0.362060546875;

            kFrontRightXPosInches = 8.8;
            kFrontRightYPosInches = -8.8;

            // Back Left
            kBackLeftDriveMotorId = 5;
            kBackLeftSteerMotorId = 7;
            kBackLeftEncoderId = 17;
            kBackLeftEncoderOffset = -0.1640625;

            kBackLeftXPosInches = -8.8;
            kBackLeftYPosInches = 8.8;

            // Back Right
            kBackRightDriveMotorId = 1;
            kBackRightSteerMotorId = 3;
            kBackRightEncoderId = 13;
            kBackRightEncoderOffset = 0.44140625;

            kBackRightXPosInches = -8.8;
            kBackRightYPosInches = -8.8;



            FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
            FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
            BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
            BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

        }       
    }
}
