package frc.robot.ConstantsFolder;


import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Camera.CameraPositions;
import frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class ConstantsBase {
    public RobotType ROBOT_TYPE = RobotType.Comp;
    public boolean IS_SIMULATION = false;

    private Auton auton;
    private Camera camera;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Arm arm;
    private Extension extension;
    private Intake intake;
    private Feeder feeder;
    private Lights lights;
    private Climber climber;

    public void setAllConstants() {
        CamBotConstants camBotConstants = new CamBotConstants();
        PracticeBotConstants practiceBotConstants = new PracticeBotConstants();
        CompBotConstants compBotConstants = new CompBotConstants();

        if (ROBOT_TYPE == RobotType.Comp) {

            this.auton = compBotConstants.new Auton();
            this.camera = compBotConstants.new Camera();
            this.arm = compBotConstants.new Arm();
            this.extension = compBotConstants.new Extension();
            this.drivetrain = compBotConstants.new Drivetrain();
            this.shooter = compBotConstants.new Shooter();
            this.intake = compBotConstants.new Intake();
            this.feeder = compBotConstants.new Feeder();
            this.lights = compBotConstants.new Lights();
            this.climber = practiceBotConstants.new Climber();

        } else if (ROBOT_TYPE == RobotType.Practice) {
            
            this.auton = practiceBotConstants.new Auton();
            this.camera = practiceBotConstants.new Camera();
            this.arm = practiceBotConstants.new Arm();
            this.extension = practiceBotConstants.new Extension();
            this.drivetrain = practiceBotConstants.new Drivetrain();
            this.shooter = practiceBotConstants.new Shooter();
            this.intake = practiceBotConstants.new Intake();
            this.feeder = practiceBotConstants.new Feeder();
            this.lights = practiceBotConstants.new Lights();
            this.climber = practiceBotConstants.new Climber();

        } else {
            this.auton = camBotConstants.new Auton();
            this.camera = camBotConstants.new Camera();
            this.drivetrain = camBotConstants.new Drivetrain();
            this.shooter = practiceBotConstants.new Shooter();
            this.intake = practiceBotConstants.new Intake();
            this.feeder = practiceBotConstants.new Feeder();
            this.lights = practiceBotConstants.new Lights();
            this.climber = practiceBotConstants.new Climber();
        }
    }

    public Intake getIntakeConstants() {
        return this.intake;
    }
    
    public Auton getAutonConstants() {
        return this.auton;
    }
    public Feeder getFeederConstants() {
        return this.feeder;
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

    public Arm getArmConstants(){
        return this.arm;
    }

    public Extension getExtensionConstants(){
        return this.extension;
    }

    public Lights getLightsConstants() {
        return this.lights;
    }

    public Climber getClimberConstants(){
        return this.climber;
    }

    private enum RobotType {
        Comp,
        Practice,
        Camera
    }


    public abstract class Auton {
        public double AUTON_DEFAULT_MAX_VELOCITY_METERS = 4;
        public double AUTON_DEFAULT_MAX_ACCEL_METERS = 3;
        
        public Auton getAuton() {
            return this;
        }
    }

    public abstract class Lights {
        public int LIGHTS_CAN_RIGHT = 51;
        public int LIGHTS_CAN_LEFT = 52;
    }

    public abstract class Climber {
        public int CLIMBER_CAN = 18;
        public double CLIMBER_SPEED = 0.1;
        public double CLIMBERPOS = 5;
    }

    public abstract class Intake {
        // intake
        public int INTAKE_ENTER_CAN = 14;
        public double INTAKESPEED = 83;
        public double INTAKE_VELOCITY_ERROR = .1;
        public double P0IntakeEnter = 4.0;
        public double I0IntakeEnter = 0.5;
        public double D0IntakeEnter = 0.0001;
        public double V0IntakeEnter = 0.12;
        public double P1IntakeEnter = 5;
        public double I1IntakeEnter = 1;
        public double D1IntakeEnter = 0.001;

        // slurper
        public double SLURPER_ARM_CANCODER_OFFSET = -44;
        public int SLURPER_ARM_CAN = 15;
        public int SLURPER_ROLLER_CAN = 20;
        public int SLURPER_CANCODER_CAN = 46;

        public double SLURPER_DOWN_ANGLE = -5;
        public double SLURPER_UP_ANGLE = 0;
        public double SLURPER_ROLLER_SPEED_RPS = 10;

        public double SLURPER_ARM_CRUISE_VELOCITY = 5;
        public double SLURPER_ARM_ACCELERATION = 10;
        public double SLURPER_ARM_JERK = 50;

        public Slot0Configs SLURPER_ARM_GAINS = new Slot0Configs()
        .withKP(60).withKI(0).withKD(0.1)
        .withKS(0.12).withKV(0.25).withKA(0);

        public Slot0Configs SLURPER_ROLLER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
        public int GRABBER_ENTER_CAN = 50;
        public double GRABBERSPEED = 40;
        public double GRABBER_VELOCITY_ERROR = .1;
        public double P0GrabberEnter = 4.0;
        public double I0GrabberEnter = 0.5;
        public double D0GrabberEnter = 0.0001;
        public double V0GrabberEnter = 0.12;
        public double P1GrabberEnter = 5;
        public double I1GrabberEnter = 1;
        public double D1GrabberEnter = 0.001;
    }

    public abstract class Feeder {
        public int FEEDER_CAN = 13;
        public double FEEDERSPEED = 83;
        public double FEEDERSPEED2 = 83;
        public int DESIREDENCODERED = 3;
        public int FEEDER_SENSOR_CHANNEL = 0;
        public double FEEDER_VELOCITY_ERROR = .3;
        public double P0IntakeFeeder = 4.0;
        public double I0IntakeFeeder = 0.5;
        public double D0IntakeFeeder = 0.0001;
        public double V0IntakeFeeder = 0.12;
        public double P1IntakeFeeder = 5;
        public double I1IntakeFeeder = 1;
        public double D1IntakeFeeder = 0.001;
        }

    public class CameraConstant{

        public CameraConstant(String name, 
                              Translation3d relativeTranslation, 
                              Rotation3d relativeRotation, 
                              Matrix<N3, N1> singleTagStdDevs,
                              Matrix<N3, N1> multiTagStdDevs) {
            this.name = name;
            this.transform = new Transform3d(relativeTranslation, relativeRotation);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;
        }

        private Matrix<N3, N1> singleTagStdDevs;
        public Matrix<N3, N1> getSingleTagStdDevs() {
            return singleTagStdDevs;
        }

        private Matrix<N3, N1> multiTagStdDevs;
        public Matrix<N3, N1> getMultiTagStdDevs() {
            return multiTagStdDevs;
        }

        private String name;
        public String getName() {
            return name;
        }
        private Transform3d transform;
        public Transform3d getTransform() {
            return transform;
        }
    }

    public abstract class Camera {
        //DEFAULT VALUES ARE PRACTICE BOT VALUES

        //Intake (Front): 12.483in forward of robot middle. On center. 8.625in above ground Perpendicular to floor
        //Shooting (Back): 12.425in reward of robot middle. On center. 6.008in above ground. Angled 8 degrees up from floor
        //Left Side: 10.696in left of robot middle. 30 degrees left of rear. 16.838in above ground. angled 5 degrees up from floor
        //Right Side: 10.696in right of robot middle. 30 degrees right of rear. 16.838in above ground. angled 5 degrees up from floor
        //(X, Y, Z)
        //X: Front and back (Front +)
        //Y: Left and right (Left +)
        //Z: Vertical distance from the floor to the camera (Up +)

        public double[] STDEV_GAIN = new double[] {.7, .7, .5};
        public double MAX_DISTANCE = 5.5;
        
        public Map<CameraPositions, CameraConstant> cameraConstants = null;
        
        public Camera(){
            cameraConstants = new EnumMap<>(CameraPositions.class);         
                cameraConstants.put(CameraPositions.FRONT, new CameraConstant("front_camera",
                new Translation3d(Units.inchesToMeters(12.483), Units.inchesToMeters(0), Units.inchesToMeters(8.625)),
                new Rotation3d(0, 0, 0),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)));

            
            cameraConstants.put(CameraPositions.BACK, new CameraConstant("back_camera",
                    new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(0), Units.inchesToMeters(6.193)),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)),
                    VecBuilder.fill(4, 4, 8),
                    VecBuilder.fill(0.5, 0.5, 1)));

        }
    }
 
    public abstract class Shooter {  
        public int RIGHT_FLYWHEEL_CAN = 12;
        public int LEFT_FLYWHEEL_CAN = 11;
        public int FEEDER_CAN = 13;

        public double RIGHT_FLYWHEEL_TARGET_RPM = 3000;
        public double LEFT_FLYWHEEL_TARGET_RPM = 3700;

        public double RIGHT_FLYWHEEL_SLOW_RPM = 1400;
        public double LEFT_FLYWHEEL_SLOW_RPM = 1400;

        public double RIGHT_FLYWHEEL_IDLE_RPM = 2000;
        public double LEFT_FLYWHEEL_IDLE_RPM = 2000;

        public double RIGHT_FLYWHEEL_PRELOAD_RPM = 2500;
        public double LEFT_FLYWHEEL_PRELOAD_RPM = 2500;



        public double TARGET_SPEED_INCREMENT = 5;
        public double START_TARGET_SPEED = 70;

        public double FEEDER_SPEED = 10;
        public double FEEDER_REVOLUTIONS = 25;

        public double FLYWHEEL_MAX_SPEED = 0.05; // percent of full speed
        public double FLYWHEEL_MAX_VELOCITY_ERROR = .0005; // percent of full speed

        public double FLYWHEEL_KP = 22.0;
        public double FLYWHEEL_KI = 0.0;
        public double FLYWHEEL_KD = 2.0;
        public double LEFT_FLYWHEEL_KV = .130; //.133
        public double RIGHT_FLYWHEEL_KV = .138; //.138
        public double LEFT_FLYWHEEL_KS = 0.8; //.384
        public double RIGHT_FLYWHEEL_KS = 0.8; //38
        // public double FLYWHEEL_PEAK_VOLTAGE = 12;
        public double FEEDER_KP = 0.25;
        public double FEEDER_KI = 0.5;
        public double FEEDER_KD = 0.0001;

        public double RFLYWHEEL_KP = 16.0;
        public double RFLYWHEEL_KI = 0.0;
        public double RFLYWHEEL_KD = 4.0;
    }

    public abstract class Arm{
        public int CANCODER_CAN = 44;
        public int ARM_CAN = 9;
        public double CRUISEVELOCITY = 400;
        public double ACCELERATION = 400;
        public double JERK = 1000;
        public double ARMKP = 250;
        public double ARMKI = 0;
        public double ARMKD = 200;
        public double ARMKV = 0.8;
        public double ARMKS = 0.1;
        public double ZERO = 95.0;
        public double SHOOT = 118.0;
        public double TRAP = 138.0;
        public double CLOSE = 151.0;
        public double PROTECT = 126.0;
        public double AMP = 139.0;
        public double HAILMARY = 140.0;
        public double ARMOFFSET = 0.4;
        public double HANDOFF = 168;
        public double BLUEDISTANCE1 = 1.16;
        public double BLUEDISTANCE2 = 2.5;
        public double BLUEDISTANCE3 = 4;
        public double BLUEDISTANCE4 = 5.3;
        public double BLUEDISTANCE5 = 6.5;
        public double BLUEDISTANCE6 = 8.8;
        public double BLUEANGLE1 = 151;
        public double BLUEANGLE2 = 134;
        public double BLUEANGLE3 = 124;
        public double BLUEANGLE4 = 119;
        public double BLUEANGLE5 = 118;
        public double BLUEANGLE6 = 117;
        public double REDDISTANCE1 = 1.16;
        public double REDDISTANCE2 = 2.5;
        public double REDDISTANCE3 = 4;
        public double REDDISTANCE4 = 5.3;
        public double REDDISTANCE5 = 6.5;
        public double REDDISTANCE6 = 8.8;
        public double REDANGLE1 = 151;
        public double REDANGLE2 = 134;
        public double REDANGLE3 = 124;
        public double REDANGLE4 = 119;
        public double REDANGLE5 = 118;
        public double REDANGLE6 = 117;
    }

    public abstract class Extension{
        public double ECRUISEVELOCITY = 15;
        public double EACCELERATION = 15;
        public double EJERK = 50;
        public double EKP = 30;
        public double EKI = 0.5;
        public double EKD = 0;
        public double EKV = 0.12;
        public double EKS = 0.25;
        public int EXTENSIONCAN = 10;
        public double SHOOTERENCODER = 5;

    }

    public abstract class Drivetrain {
        public double CAM_MAX_ERROR = 1.0;

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


        public Slot0Configs TELEOP_SWERVE_STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);

        public Slot0Configs TELEOP_SWERVE_DRIVE_GAINS = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

        public TorqueCurrentConfigs AUTON_STEER_CURRENT = new TorqueCurrentConfigs()
                                    .withPeakForwardTorqueCurrent(300)
                                    .withPeakReverseTorqueCurrent(-300);
        public TorqueCurrentConfigs AUTON_DRIVE_CURRENT = new TorqueCurrentConfigs()
                                    .withPeakForwardTorqueCurrent(300)
                                    .withPeakReverseTorqueCurrent(-300);

        public TorqueCurrentConfigs TELEOP_STEER_CURRENT = new TorqueCurrentConfigs()
                                    .withPeakForwardTorqueCurrent(70)
                                    .withPeakReverseTorqueCurrent(-70);
        public TorqueCurrentConfigs TELEOP_DRIVE_CURRENT = new TorqueCurrentConfigs()
                                    .withPeakForwardTorqueCurrent(150)
                                    .withPeakReverseTorqueCurrent(-150);

        public double WHEEL_SLIP_CURRENT = 500.0; // *tune later

        // Meters per second theroretical max speed at 12 volts
        public double FREE_SPEED_12V = 8.37032; // 6.37032

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