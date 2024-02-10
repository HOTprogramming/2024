package frc.robot.ConstantsFolder;


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

public class ConstantsBase {
    public RobotType ROBOT_TYPE = RobotType.Practice;
    public boolean IS_SIMULATION = true;

    private Auton auton;
    private Camera camera;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;

    public void setAllConstants() {
        CamBotConstants camBotConstants = new CamBotConstants();
        PracticeBotConstants practiceBotConstants = new PracticeBotConstants();
        CompBotConstants compBotConstants = new CompBotConstants();

        if (ROBOT_TYPE == RobotType.Comp) {

            this.auton = compBotConstants.new Auton();
            this.camera = compBotConstants.new Camera();
            this.drivetrain = compBotConstants.new Drivetrain();
            this.shooter = practiceBotConstants.new Shooter();
            this.intake = practiceBotConstants.new Intake();

        } else if (ROBOT_TYPE == RobotType.Practice) {
            
            this.auton = practiceBotConstants.new Auton();
            this.camera = practiceBotConstants.new Camera();
            this.drivetrain = practiceBotConstants.new Drivetrain();
            this.shooter = practiceBotConstants.new Shooter();
            this.intake = practiceBotConstants.new Intake();

        } else {
            this.auton = camBotConstants.new Auton();
            this.camera = camBotConstants.new Camera();
            this.drivetrain = camBotConstants.new Drivetrain();
            this.shooter = practiceBotConstants.new Shooter();
            this.intake = practiceBotConstants.new Intake();
        }
    }

    public Intake getIntakeConstants() {
        return this.intake;
    }
    
    public Auton getAutonConstants() {
        return this.auton;
    }

    public Camera getCameraConstants() {
        return this.camera;
    }

    public Drivetrain getDriveTrainConstants() {
        return this.drivetrain;
    }

    public Shooter getShooterConstants() {
        return this.shooter;
    }


    private enum RobotType {
        Comp,
        Practice,
        Camera
    }


    public abstract class Auton {
        public double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
        public double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;
        
        public Auton getAuton() {
            return this;
        }
    }

    public abstract class Intake {
        public int INTAKE_ENTER_CAN = 14;
        public int INTAKE_TRANSFER_CAN = 13;
        public double INTAKESPEED = 83;
        public double INTAKESTOP = 0;
        public double INTAKE_VELOCITY_ERROR = .1;
        public int ENTER_SENSOR_CHANNEL = 0;
        public int TRANSFER_SENSOR_CHANNEL = 1;
        public double P0IntakeEnter = 8.0;
        public double I0IntakeEnter = 0.5;
        public double D0IntakeEnter = 0.0001;
        public double V0IntakeEnter = 0.12;
        public double P1IntakeEnter = 5;
        public double I1IntakeEnter = 1;
        public double D1IntakeEnter = 0.001;
        public double EnterPeakForwardTorqueCurrent = 40;
        public double EnterPeakReverseTorqueCurrent = -40;
        public double P0IntakeTransfer = 0.11;
        public double I0IntakeTransfer = 0.5;
        public double D0IntakeTransfer = 0.0001;
        public double V0IntakeTransfer = 0.12;
        public double P1IntakeTransfer = 5;
        public double I1IntakeTransfer = 1;
        public double D1IntakeTransfer = 0.001;
        public double TransferPeakForwardTorqueCurrent = 40;
        public double TransferPeakReverseTorqueCurrent = -40;
    }
  
  
    public abstract class Camera {
        public boolean HAS_CAMERA = false;
        public String FRONT_CAMERA_NAME = "front_camera";
        
        public Translation3d FRONT_CAMERA_REALITIVE_POSITION = new Translation3d(.35, .29, .165);
        public Rotation3d FRONT_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-8), 0);
        public Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(FRONT_CAMERA_REALITIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION);

        public String REAR_CAMERA_NAME = "back_camera";

        public Translation3d REAR_CAMERA_REALITIVE_POSITION = new Translation3d(0, -.3, .1);
        public Rotation3d REAR_CAMERA_RELATIVE_ROTATION = new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180));
        public Transform3d REAR_CAMERA_TRANSFORM = new Transform3d(REAR_CAMERA_REALITIVE_POSITION, REAR_CAMERA_RELATIVE_ROTATION);

    }
 
    public abstract class Shooter {
        public int RIGHT_FLYWHEEL_CAN = 12;
        public int LEFT_FLYWHEEL_CAN = 11;
        public int FEEDER_CAN = 13;

        public double TARGET_SPEED_INCREMENT = 5;
        public double START_TARGET_SPEED = 70;

        public double FEEDER_SPEED = 10;
        public double FEEDER_REVOLUTIONS = 25;

        public double FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
        public double FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed

        public double FLYWHEEL_KP = 0.25;
        public double FLYWHEEL_KI = 0.5;
        public double FLYWHEEL_KD = 0.0001;
        public double LEFT_FLYWHEEL_KV = .130; //.133
        public double RIGHT_FLYWHEEL_KV = .130; //.138
        public double LEFT_FLYWHEEL_KS = 0.8; //.384
        public double RIGHT_FLYWHEEL_KS = 0.8; //38
        public double FLYWHEEL_PEAK_VOLTAGE = 12;
        public double FEEDER_KP = 0.25;
        public double FEEDER_KI = 0.5;
        public double FEEDER_KD = 0.0001;

        public double RFLYWHEEL_KP = 0.25;
        public double RFLYWHEEL_KI = 0.5;
        public double RFLYWHEEL_KD = 0.0001;
    }

    public abstract class Drivetrain {
        public double AUTON_DEFAULT_MAX_VELOCITY_METERS = 5;
        public double AUTON_DEFAULT_MAX_ACCEL_METERS = 2;

        public double ROBOT_LENGTH_INCHES = 20.25;
        public double ROBOT_WITDTH_INCHES = 20.25;
        public double MAX_VELOCITY_METERS = 6.37032; // from SDS
        // public double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / Math.hypot(Units.inchesToMeters(ROBOT_LENGTH_INCHES / 2), Units.inchesToMeters(ROBOT_WITDTH_INCHES / 2));
        // public double MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2; // fix latr 0.7274007458
        public double MAX_ANGULAR_VELOCITY_RADS = MAX_VELOCITY_METERS / 0.7274007458;

        // WCS Docs X3 11 https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options 
        // SWERVE BUILDER
        public Slot0Configs SWERVE_STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);

        public Slot0Configs SWERVE_DRIVE_GAINS = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
        
        // private ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

        // private ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.TorqueCurrentFOC;

        
        public ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

        public ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;



        public double WHEEL_SLIP_CURRENT = 300.0; // *tune later

        // Meters per second theroretical max speed at 12 volts
        public double FREE_SPEED_12V = 6.37032;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public double kCoupleRatio = 3.5714285714285716; // tune later

        public double kDriveGearRatio = 5.01;
        public double kSteerGearRatio = 13.3714;
        public double kWheelRadiusInches = 2;

        // Are steer motors GENERALLY reversed
        public boolean kSteerMotorReversed = true;


        

        public String CANBUS_NAME = "drivetrain";
        public int PIDGEON_CAN = 50;

        // These are only used for simulation
        public double kSteerInertia = 0.00001;
        // REAL swerve rotational inertia 3.05 lbs*in^2 or 0.0008925509 kg * m^2
        public double kDriveInertia = 0.001;

        // Simulated voltage necessary to overcome friction
        public double kSteerFrictionVoltage = 0.25;
        public double kDriveFrictionVoltage = 0.25;   


        public SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
        .withPigeon2Id(PIDGEON_CAN)
        .withCANbusName(CANBUS_NAME);

        public SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
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
            public boolean SWERVE_FRONT_LEFT_DRIVE_UNINVERT = !true;
            public boolean SWERVE_FRONT_LEFT_STEER_UNINVERT = false;
            public int kFrontLeftDriveMotorId = 8;
            public int kFrontLeftSteerMotorId = 7;
            public int kFrontLeftEncoderId = 43;
            public double kFrontLeftEncoderOffset = 0.440673828125 * Math.PI;

            public double kFrontLeftXPosInches = 10.125;
            public double kFrontLeftYPosInches = 10.125;

            // Front Right
            public boolean SWERVE_FRONT_RIGHT_DRIVE_UNINVERT = !true;
            public boolean SWERVE_FRONT_RIGHT_STEER_UNINVERT = true;
            public int kFrontRightDriveMotorId = 4;
            public int kFrontRightSteerMotorId = 3;
            public int kFrontRightEncoderId = 41;
            public double kFrontRightEncoderOffset = 0.098876953125 * Math.PI;

            public double kFrontRightXPosInches = 10.125;
            public double kFrontRightYPosInches = -10.125;

            // Back Left
            public boolean SWERVE_BACK_LEFT_DRIVE_UNINVERT = !false;
            public boolean SWERVE_BACK_LEFT_STEER_UNINVERT = false;
            public int kBackLeftDriveMotorId = 6;
            public int kBackLeftSteerMotorId = 5;
            public int kBackLeftEncoderId = 42;
            public double kBackLeftEncoderOffset = -0.450439453125 * Math.PI;

            public double kBackLeftXPosInches = -10.125;
            public double kBackLeftYPosInches = 10.125;


            // Back Right
            public boolean SWERVE_BACK_RIGHT_DRIVE_UNINVERT = !true;
            public boolean SWERVE_BACK_RIGHT_STEER_UNINVERT = false;
            public int kBackRightDriveMotorId = 2;
            public int kBackRightSteerMotorId = 1;
            public int kBackRightEncoderId = 40;
            public double kBackRightEncoderOffset = -0.44140625 * Math.PI;

            public double kBackRightXPosInches = -10.125;
            public double kBackRightYPosInches = -10.125;


            public SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset / Math.PI, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), !SWERVE_FRONT_LEFT_DRIVE_UNINVERT)
            .withSteerMotorInverted(!SWERVE_FRONT_LEFT_STEER_UNINVERT);
            public SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset / Math.PI, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), !SWERVE_FRONT_RIGHT_DRIVE_UNINVERT)
            .withSteerMotorInverted(!SWERVE_FRONT_RIGHT_STEER_UNINVERT);
            public SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset / Math.PI, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), !SWERVE_BACK_LEFT_DRIVE_UNINVERT)
            .withSteerMotorInverted(!SWERVE_BACK_LEFT_STEER_UNINVERT);
            public SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset / Math.PI, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), !SWERVE_BACK_RIGHT_DRIVE_UNINVERT)
            .withSteerMotorInverted(!SWERVE_BACK_RIGHT_STEER_UNINVERT);
    }
}