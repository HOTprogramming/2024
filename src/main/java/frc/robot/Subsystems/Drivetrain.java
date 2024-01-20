package frc.robot.Subsystems;

import static frc.robot.Constants.Drivetrain.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.RobotCommander.DriveMode;
import frc.robot.trajectory.CustomHolonomicDriveController;
import frc.robot.trajectory.RotationSequence;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends SwerveDrivetrain implements SubsystemBase {
    RobotState robotState;

    // swerve commands
    private static final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.Idle coast = new SwerveRequest.Idle();
    private static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private static final SwerveRequest.ApplyChassisSpeeds withChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
    

    // NetworkTables
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Drivetrain");
    StringPublisher fieldTypePublisher = table.getStringTopic(".type").publish();
    DoubleArrayPublisher posePublisher = table.getDoubleArrayTopic("RobotPose").publish();
    DoubleArrayPublisher velocityPublisher = table.getDoubleArrayTopic("RobotVelocity").publish();
    

    // for velocity calcs
    private SwerveDriveState currentState;
    private Pose2d lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();
    private Translation2d velocities = new Translation2d(0, Rotation2d.fromDegrees(0));


    // Drive controllers
    private static final PIDController xController = new PIDController(4.9, 0.1, .1);
    private static final PIDController yController = new PIDController(4.9, 0.1, .1);
    private static final PIDController thetaController = new PIDController(5, 0.1, .1);

    private static final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(xController, yController, thetaController);


    public Drivetrain(RobotState robotState) {
        super(DRIVETRAIN_CONSTANTS, 
        FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS, BACK_RIGHT_MODULE_CONSTANTS);

        this.robotState = robotState;    

        configNeutralMode(NeutralModeValue.Brake);
        
    }

    private void percentDrive(double[] drivePercents) {
        SmartDashboard.putNumber("XPercentTarget", drivePercents[0]);
        SmartDashboard.putNumber("YPercentTarget", drivePercents[1]);
        SmartDashboard.putNumber("ThetaPercentTarget", drivePercents[2]);

        setControl(fieldCentric.withVelocityX(drivePercents[0] * MAX_VELOCITY_METERS)
                                .withVelocityY(drivePercents[1] * MAX_VELOCITY_METERS)
                                .withRotationalRate(drivePercents[2] * MAX_ANGULAR_VELOCITY_RADS));
                                
    }

    private void stateDrive(State holoDriveState, RotationSequence.State rotationState) {
        ChassisSpeeds chassisSpeeds = driveController.calculate(getState().Pose, holoDriveState, rotationState);

        setControl(withChassisSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public void updateState() {
        currentState = getState();
        
        if (driveController.atReference()) {
            robotState.setAtTargetPose(true);
        } else {
            robotState.setAtTargetPose(false);
        }

        if (currentState.Pose != null) {
            double currentTime = Utils.getCurrentTimeSeconds();
            double timeDifference = currentTime - lastTime;
            lastTime = currentTime;
            Translation2d distanceDifference = currentState.Pose.minus(lastPose).getTranslation();
            lastPose = currentState.Pose;

            velocities = distanceDifference.div(timeDifference);

            fieldTypePublisher.set("Field2d");
            posePublisher.set(new double[] {
                currentState.Pose.getX(),
                currentState.Pose.getY(),
                currentState.Pose.getRotation().getDegrees()
            });

            velocityPublisher.set(new double[] {
                velocities.getX(),
                velocities.getY(),
                velocities.getAngle().getDegrees()
            });
        }

        if (currentState.ModuleStates != null) {
            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumber("Swerve Encoder " + i + " (rads)", currentState.ModuleStates[i].angle.getRadians());
            }
        }
    }

    @Override
    public void init(RobotCommander commander) {}
    
    @Override
    public void enabled(RobotCommander commander) {
        // if (commander.getDriveMode() == DriveMode.percent) {
        //     percentDrive(commander.getDrivePercentCommand());
        // } else if (commander.getDriveMode() == DriveMode.stateDrive) {
        //     stateDrive(commander.getDriveState(), commander.getDriveRotationState());
        // }

        percentDrive(commander.getDrivePercentCommand());
        
    }

    @Override
    public void disabled() {
        setControl(brake);
    }

    @Override
    public void reset() {}

   
}
