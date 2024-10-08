package frc.robot.ConstantsFolder;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Camera.CameraPositions;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class PracticeBotConstants extends ConstantsBase {

    public class Auton extends ConstantsBase.Auton {
        public Auton() {
            AUTON_DEFAULT_MAX_VELOCITY_METERS = 4.0;
            AUTON_DEFAULT_MAX_ACCEL_METERS = 3.0;
        }        
    }
  
  
    public class Camera extends ConstantsBase.Camera {
        public Camera() {
            super();
            cameraConstants.put(CameraPositions.LEFT,  new CameraConstant("left_camera",
                                                       new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(-4), Units.inchesToMeters(16.838)),
                                                       new Rotation3d(Units.degreesToRadians(-5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(120)),
                                                       VecBuilder.fill(4, 4, 8),
                                                       VecBuilder.fill(0.5, 0.5, 1)));

            cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("right_camera",
                                                       new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(11), Units.inchesToMeters(16.838)),
                                                       new Rotation3d(Units.degreesToRadians(5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(-120)),
                                                       VecBuilder.fill(4, 4, 8),
                                                       VecBuilder.fill(0.5, 0.5, 1)));
        }
    }

    public class Intake extends ConstantsBase.Intake {
        public Intake() {
         SLURPER_ARM_CANCODER_OFFSET = -145;
         INTAKE_ENTER_CAN = 14;
         INTAKESPEED = 83;
         INTAKE_VELOCITY_ERROR = .01;
         GRABBER_ENTER_CAN = 50;
         GRABBERSPEED = 83;
         GRABBER_VELOCITY_ERROR = .01;
        }
    }
    
    public class Feeder extends ConstantsBase.Feeder {
        public Feeder() {
         FEEDER_CAN = 13;
         FEEDERSPEED = 83;
         FEEDERSPEED2 = 83;
         DESIREDENCODERED = 7;
         FEEDER_VELOCITY_ERROR = .01;
         FEEDER_SENSOR_CHANNEL = 0;
        }
    }

    public class Lights extends ConstantsBase.Lights {
        public Lights() {
         LIGHTS_CAN_RIGHT = 51;
        }
    }


    public class Climber extends ConstantsBase.Climber {
        public Climber() {
            CLIMBER_CAN = 18;
            CLIMBER_SPEED = 0.1;
        }
    } 

    public class Shooter extends ConstantsBase.Shooter {
        public Shooter() {
            RIGHT_FLYWHEEL_CAN = 12;
            LEFT_FLYWHEEL_CAN = 11;
            FEEDER_CAN = 13;

            RIGHT_FLYWHEEL_TARGET_RPM = 3000;
            LEFT_FLYWHEEL_TARGET_RPM = 3700;
    
            TARGET_SPEED_INCREMENT = 5;
            START_TARGET_SPEED = 70;
    
            FEEDER_SPEED = 10;
            FEEDER_REVOLUTIONS = 25;
    
            FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
            FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed
    
            FLYWHEEL_KP = 22.0;
            FLYWHEEL_KI = 0.0;
            FLYWHEEL_KD = 2.0;
            LEFT_FLYWHEEL_KV = .130; //.133
            RIGHT_FLYWHEEL_KV = .138; //.138
            LEFT_FLYWHEEL_KS = 0.8; //.384
            RIGHT_FLYWHEEL_KS = 0.8; //38
            FEEDER_KP = 0.25;
            FEEDER_KI = 0.5;
            FEEDER_KD = 0.0001;
    
            RFLYWHEEL_KP = 16.0;
            RFLYWHEEL_KI = 0.0;
            RFLYWHEEL_KD = 4.0;
        }

    }

    public class Arm extends ConstantsBase.Arm {
       
        public Arm(){
         CANCODER_CAN = 44;
         ARM_CAN = 9;
         CRUISEVELOCITY = 500;
         ACCELERATION = 420;//350
         JERK = 1000;
         ARMKP = 200;
         ARMKI = 20;
         ARMKD = 0;//4
         ARMKV = 0.8;
         ARMKS = 0.4;
         ZERO = 95.0;
         SHOOT = 118.0;
         TRAP = 138.0;
         CLOSE = 151.0;
         PROTECT = 126.0;
         AMP = 120.3; //was 143.3; 133.3
         HANDOFF = 160;
         ARMOFFSET = 0.4;
         REDDISTANCE1 = 1.16;
         REDDISTANCE2 = 2.5;
         REDDISTANCE3 = 4;
         REDDISTANCE4 = 5.3;
         REDDISTANCE5 = 6.5;
         REDANGLE1 = 151;
         REDANGLE2 = 134;
         REDANGLE3 = 124;
         REDANGLE4 = 119;
         REDANGLE5 = 118;
        }
    }

    public class Extension extends ConstantsBase.Extension{
        public Extension(){
        ECRUISEVELOCITY = 15;
        EACCELERATION = 15;
        EJERK = 50;
        EKP = 30;
        EKI = 0.5;
        EKD = 0;
        EKV = 0.12;
        EKS = 0.25;
        SHOOTERENCODER = 7;
        }
    }

    public class Drivetrain extends ConstantsBase.Drivetrain {
        public Drivetrain() {
            AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
            AUTON_DEFAULT_MAX_ACCEL_METERS = 2;

            ROBOT_LENGTH_INCHES = 20.25;
            ROBOT_WITDTH_INCHES = 20.25;
            MAX_VELOCITY_METERS = 6.37032; // from SDS
            // MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
            // MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
            MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / 0.7274007458;

            // WCS Docs X3 11 https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options 
            // SWERVE BUILDER
            SWERVE_STEER_GAINS = new Slot0Configs()
            .withKP(100).withKI(0).withKD(.2)
            .withKS(0).withKV(1.5).withKA(0);

            SWERVE_DRIVE_GAINS = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);
            
            // STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

            // DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

            if (IS_SIMULATION) {
                STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

                DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
            } else {
                STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

                DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;
            }
            



            WHEEL_SLIP_CURRENT = 600.0; //300

            // Meters per second theroretical max speed at 12 volts
            FREE_SPEED_12V = 6.37032;

            // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
            // This may need to be tuned to your individual robot
            kCoupleRatio = 3.5714285714285716; // tune later

            kDriveGearRatio = 5.01;
            kSteerGearRatio = 13.3714;
            kWheelRadiusInches = 2;

            // Are steer motors GENERALLY reversed
            kSteerMotorReversed = true;


            

            CANBUS_NAME = "drivetrain";
            PIDGEON_CAN = 50;

            // These are only used for simulation
            kSteerInertia = 0.00001;
            // REAL swerve rotational inertia 3.05 lbs*in^2 or 0.0008925509 kg * m^2
            kDriveInertia = 0.001;

            // Simulated voltage necessary to overcome friction
            kSteerFrictionVoltage = 0.25;
            kDriveFrictionVoltage = 0.25;   


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
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);


                        
            
                // offsets in radians

                // Front Left
                SWERVE_FRONT_LEFT_DRIVE_UNINVERT = !true;
                SWERVE_FRONT_LEFT_STEER_UNINVERT = false;
                kFrontLeftDriveMotorId = 1;
                kFrontLeftSteerMotorId = 2;
                kFrontLeftEncoderId = 43;
                kFrontLeftEncoderOffset = 0.440673828125 * Math.PI;

                kFrontLeftXPosInches = 10.125;
                kFrontLeftYPosInches = 10.125;

                // Front Right
                SWERVE_FRONT_RIGHT_DRIVE_UNINVERT = !true;
                SWERVE_FRONT_RIGHT_STEER_UNINVERT = true;
                kFrontRightDriveMotorId = 3;
                kFrontRightSteerMotorId = 4;
                kFrontRightEncoderId = 41;
                kFrontRightEncoderOffset = 0.098876953125 * Math.PI;

                kFrontRightXPosInches = 10.125;
                kFrontRightYPosInches = -10.125;

                // Back Left
                SWERVE_BACK_LEFT_DRIVE_UNINVERT = !false;
                SWERVE_BACK_LEFT_STEER_UNINVERT = false;
                kBackLeftDriveMotorId = 5;
                kBackLeftSteerMotorId = 6;
                kBackLeftEncoderId = 42;
                kBackLeftEncoderOffset = -0.450439453125 * Math.PI;

                kBackLeftXPosInches = -10.125;
                kBackLeftYPosInches = 10.125;


                // Back Right
                SWERVE_BACK_RIGHT_DRIVE_UNINVERT = !true;
                SWERVE_BACK_RIGHT_STEER_UNINVERT = false;
                kBackRightDriveMotorId = 7;
                kBackRightSteerMotorId = 8;
                kBackRightEncoderId = 40;
                kBackRightEncoderOffset = -0.44140625 * Math.PI;

                kBackRightXPosInches = -10.125;
                kBackRightYPosInches = -10.125;


                FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset / Math.PI, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), !SWERVE_FRONT_LEFT_DRIVE_UNINVERT)
                .withSteerMotorInverted(!SWERVE_FRONT_LEFT_STEER_UNINVERT);
                FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset / Math.PI, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), !SWERVE_FRONT_RIGHT_DRIVE_UNINVERT)
                .withSteerMotorInverted(!SWERVE_FRONT_RIGHT_STEER_UNINVERT);
                BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset / Math.PI, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), !SWERVE_BACK_LEFT_DRIVE_UNINVERT)
                .withSteerMotorInverted(!SWERVE_BACK_LEFT_STEER_UNINVERT);
                BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset / Math.PI, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), !SWERVE_BACK_RIGHT_DRIVE_UNINVERT)
                .withSteerMotorInverted(!SWERVE_BACK_RIGHT_STEER_UNINVERT);
            
        }
    }
}
