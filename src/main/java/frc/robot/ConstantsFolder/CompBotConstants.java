package frc.robot.ConstantsFolder;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Camera.CameraPositions;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class CompBotConstants extends ConstantsBase {

    public class Auton extends ConstantsBase.Auton {
        public Auton() {
            AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
            AUTON_DEFAULT_MAX_ACCEL_METERS = 2;
        }        
    }
  
  
    public class Camera extends ConstantsBase.Camera {
        public Camera() {
            super();
            cameraConstants.put(CameraPositions.LEFT,  new CameraConstant("left_camera",
                                                       new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(11.49), Units.inchesToMeters(16.74)),
                                                       new Rotation3d(Units.degreesToRadians(-5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(120.38)),
                                                       VecBuilder.fill(4, 4, 8),
                                                       VecBuilder.fill(0.5, 0.5, 1)));

            cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("right_camera",
                                                       new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(-11.49), Units.inchesToMeters(16.74)),
                                                       new Rotation3d(Units.degreesToRadians(5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(-120.38)),
                                                       VecBuilder.fill(4, 4, 8),
                                                       VecBuilder.fill(0.5, 0.5, 1)));
        }
    }
    public class Intake extends ConstantsBase.Intake {
        public Intake() {
            SLURPER_ARM_CANCODER_OFFSET = -44;
            INTAKE_ENTER_CAN = 14;
            INTAKESPEED = 83;
            INTAKE_VELOCITY_ERROR = .01;
            GRABBER_ENTER_CAN = 50;
            GRABBERSPEED = 83;
            GRABBER_VELOCITY_ERROR = .01;
            SLURPER_ROLLER_CAN = 16;
        }
    }
    public class Feeder extends ConstantsBase.Feeder {
        public Feeder() {
         FEEDER_CAN = 13;
         FEEDERSPEED = 90;
         FEEDERSPEED2 = 60;
         DESIREDENCODERED = 0;
         FEEDER_VELOCITY_ERROR = .01;
         FEEDER_SENSOR_CHANNEL = 0;
        }
    }
    public class Drivetrain extends ConstantsBase.Drivetrain {
        public Drivetrain() {
            ROBOT_LENGTH_INCHES = 20.25;
            ROBOT_WITDTH_INCHES = 20.25;
            MAX_VELOCITY_METERS = 7.37032; // from SDS
            // public MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
            // public MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
            MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / (0.7274007458 * .8);

            // WCS Docs X3 11 https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options 
            // SWERVE BUILDER
            SWERVE_STEER_GAINS = new Slot0Configs()
            .withKP(100).withKI(0).withKD(.2) // 400, 0, 8 , 0 ,1.5, 0
            .withKS(0).withKV(1.5).withKA(0);

            SWERVE_DRIVE_GAINS = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);
            
            
            if (IS_SIMULATION) {
                STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

                DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
            } else {
                STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

                DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage
                ;
            }

            WHEEL_SLIP_CURRENT = 650.0; // *tune later

            // Meters per second theroretical max speed at 12 volts
            FREE_SPEED_12V = 7.37032;

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
            kFrontLeftEncoderOffset = 0.3193359375 * Math.PI;

            kFrontLeftXPosInches = 10.125;
            kFrontLeftYPosInches = 10.125;

            // Front Right
            SWERVE_FRONT_RIGHT_DRIVE_UNINVERT = !true;
            SWERVE_FRONT_RIGHT_STEER_UNINVERT = true;
            kFrontRightDriveMotorId = 3;
            kFrontRightSteerMotorId = 4;
            kFrontRightEncoderId = 41;
            kFrontRightEncoderOffset = -0.149169921875 * Math.PI;

            kFrontRightXPosInches = 10.125;
            kFrontRightYPosInches = -10.125;

            // Back Left
            SWERVE_BACK_LEFT_DRIVE_UNINVERT = !false;
            SWERVE_BACK_LEFT_STEER_UNINVERT = false;
            kBackLeftDriveMotorId = 5;
            kBackLeftSteerMotorId = 6;
            kBackLeftEncoderId = 42;
            kBackLeftEncoderOffset = -0.04150390625 * Math.PI;

            kBackLeftXPosInches = -10.125;
            kBackLeftYPosInches = 10.125;


            // Back Right
            SWERVE_BACK_RIGHT_DRIVE_UNINVERT = !true;
            SWERVE_BACK_RIGHT_STEER_UNINVERT = false;
            kBackRightDriveMotorId = 7;
            kBackRightSteerMotorId = 8;
            kBackRightEncoderId = 40;
            kBackRightEncoderOffset = -0.279296875 * Math.PI;

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

    public class Shooter extends ConstantsBase.Shooter {
        public Shooter() {            
            FEEDER_KP = 0.25;
            FEEDER_KI = 0.5;
            FEEDER_KD = 0.0001;
    
            RIGHT_FLYWHEEL_TARGET_RPM = 5400;
            LEFT_FLYWHEEL_TARGET_RPM = 3000;

            RIGHT_FLYWHEEL_CAN = 12;
            LEFT_FLYWHEEL_CAN = 11;
            FEEDER_CAN = 13;
    
            TARGET_SPEED_INCREMENT = 5;
            START_TARGET_SPEED = 70;
    
            FEEDER_SPEED = 10;
            FEEDER_REVOLUTIONS = 25;
    
            FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
            FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed
    
            FLYWHEEL_KP = 12.0; // 22.0
            FLYWHEEL_KI = 0.0; // 0.0
            FLYWHEEL_KD = 0.0; // 2.0
            LEFT_FLYWHEEL_KV = 0.14; //.133
            LEFT_FLYWHEEL_KS = 10.1; // .8 | .384


            RFLYWHEEL_KP = 9.0; // 16.0
            RFLYWHEEL_KI = 0.0; // 0.0
            RFLYWHEEL_KD = 0.0; // 4.0
            RIGHT_FLYWHEEL_KV = 0.18; //.138
            RIGHT_FLYWHEEL_KS = 5.7; // 0.8 | .38




        }

    }

    public class Lights extends ConstantsBase.Lights {
        public Lights() {
         LIGHTS_CAN_RIGHT = 51;
        }
    }

    public class Arm extends ConstantsBase.Arm {
        public Arm(){
            CANCODER_CAN = 44;
            ARM_CAN = 9;
            CRUISEVELOCITY = 600; // 800
            ACCELERATION = 350;//420
            JERK = 1000;
            ARMKP = 320;
            ARMKI = 0;
            ARMKD = 0;//4
            ARMKV = 0.8;
            ARMKS = 0.4;
            ZERO = 95.7;
            SHOOT = 118.0;
            TRAP = 144.0;
            CLOSE = 148.5;
            PROTECT = 130.1;
            AMP = 139.992; //was 140.3;
            HANDOFF = 168;

            ARMOFFSET = -321.684 / 360.0; //-0.4895 rotations, now degrees 
            

            BLUEDISTANCE1 = 1.21; 
            BLUEDISTANCE2 = 2.73; //2.8
            BLUEDISTANCE3 = 3.72; //4.0
            BLUEDISTANCE4 = 5.33; //5.26
            BLUEDISTANCE5 = 8.2; 
            BLUEDISTANCE6 = 8.3;
            BLUEANGLE1 = 148.5;
            BLUEANGLE2 = 131.1; 
            BLUEANGLE3 = 126.5;  
            BLUEANGLE4 = 122.5;  
            BLUEANGLE5 = 122.4;
            BLUEANGLE6 = 122.3; 

            // BLUEANGLE1 = 148.0;
            // BLUEANGLE2 = 133.4;
            // BLUEANGLE3 = 125.5;
            // BLUEANGLE4 = 121.0;   
            // BLUEANGLE5 = 119.9;
            // BLUEANGLE6 = 117.8; 
            
            
            REDDISTANCE1 = 1.21; // BATTER 0 INCHES
            REDDISTANCE2 = 2.82; //2.8// MID RING 62 INCHES 
            REDDISTANCE3 = 3.8;//4.0 // FRONT BUMPER FLUSH WITH CLOSE TRUSS 108 in
            REDDISTANCE4 = 5.43;//26 // INTAKE BUMPER ON WING LINE 163 in
            REDDISTANCE5 = 8.2; // WHITE LINE
            REDDISTANCE6 = 8.3; // WHITE LINE
            REDANGLE1 = 148.5;
            REDANGLE2 = 131.2;
            REDANGLE3 = 126.6;
            REDANGLE4 = 123.7;
            REDANGLE5 = 122.4;  
            REDANGLE6 = 122.3;
            
            // REDANGLE1 = 148.5; // old rings
            // REDANGLE2 = 134.6;
            // REDANGLE3 = 126.7;
            // REDANGLE4 = 123.2;   
            // REDANGLE5 = 119.9;
            // REDANGLE6 = 117.8; 
        }
    }

    public class Extension extends ConstantsBase.Extension{
        public Extension(){
            ECRUISEVELOCITY = 25;
            EACCELERATION = 15;
            EJERK = 50;
            EKP = 80;
            EKI = 0.5;
            EKD = 0;
            EKV = 0.12;
            EKS = 0.25;
            SHOOTERENCODER = 2.3;
        }
    }
}