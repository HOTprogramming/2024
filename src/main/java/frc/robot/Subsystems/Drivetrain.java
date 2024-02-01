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
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SwerveDrivetrain implements SubsystemBase {
    RobotState robotState;
    boolean driveType = true;

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

    private static final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            xController, yController, thetaController);

    // TEMP pose 2d for get angle snap command
    private Pose2d snapPose = new Pose2d(0.93, 5.55, Rotation2d.fromDegrees(0));

    public Drivetrain(RobotState robotState) {
        // call swervedriveDrivetrain constructor (parent class)
        super(DRIVETRAIN_CONSTANTS,
                FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS,
                BACK_RIGHT_MODULE_CONSTANTS);

        // initalize robot state
        this.robotState = robotState;

        configNeutralMode(NeutralModeValue.Brake);
        // seedFieldRelative(new Pose2d(16.54, 8.2, Rotation2d.fromDegrees(-90)));
    }

    private void percentDrive(double[] drivePercents) {
        if (driveType) {
            setControl(fieldCentric.withVelocityX(drivePercents[0] * MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * MAX_VELOCITY_METERS)
                    .withRotationalRate(drivePercents[2] * MAX_ANGULAR_VELOCITY_RADS));
        } else {
            setControl(robotCentric.withVelocityX(drivePercents[0] * MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * MAX_VELOCITY_METERS)
                    .withRotationalRate(drivePercents[2] * MAX_ANGULAR_VELOCITY_RADS));
        }
    }

    private void autoTurnControl(double[] drivePercents, double targetTheta) {
        // Uses theta control to rotate bot (radians)
        if (driveType) {
            setControl(fieldCentric.withVelocityX(drivePercents[0] * MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * MAX_VELOCITY_METERS)
                    .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                            Units.degreesToRadians(targetTheta))));
        } else {
            setControl(robotCentric.withVelocityX(drivePercents[0] * MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * MAX_VELOCITY_METERS)
                    .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                            Units.degreesToRadians(targetTheta))));
        }

    }

    private void stateDrive(State holoDriveState, RotationSequence.State rotationState) {
        if (holoDriveState != null && rotationState != null) {
            // drive with target states from trajectory generator (auton)
            ChassisSpeeds chassisSpeeds = driveController.calculate(getState().Pose, holoDriveState, rotationState);

            setControl(withChassisSpeeds.withSpeeds(chassisSpeeds));
        }

    }

    @Override
    public void updateState() {
        // gets current drive state
        currentState = getState();

        // updates robotState for auton pathing
        if (driveController.atReference()) {
            robotState.setAtTargetPose(true);
        } else {
            robotState.setAtTargetPose(false);
        }

        // updates pose reliant functions
        if (currentState.Pose != null) {
            robotState.setDrivePose(currentState.Pose);
            double currentTime = Utils.getCurrentTimeSeconds();
            double timeDifference = currentTime - lastTime;
            lastTime = currentTime;
            Translation2d distanceDifference = currentState.Pose.minus(lastPose).getTranslation();
            lastPose = currentState.Pose;

            velocities = distanceDifference.div(timeDifference);

            // publishes pose to smartdashboard
            fieldTypePublisher.set("Field2d");
            posePublisher.set(new double[] {
                    currentState.Pose.getX(),
                    currentState.Pose.getY(),
                    currentState.Pose.getRotation().getDegrees()
            });

            // publishes velocity to smartdashboard
            velocityPublisher.set(new double[] {
                    velocities.getX(),
                    velocities.getY(),
                    velocities.getAngle().getDegrees()
            });
        }

        // updates module states for finding encoder offsets
        if (currentState.ModuleStates != null) {
            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumber("Swerve Encoder " + i + " (rads)",
                        currentState.ModuleStates[i].angle.getRadians());
            }
        }

    }

    @Override
    public void init(RobotCommander commander) {
        // sets start pose to auton start pose
        seedFieldRelative(commander.getOdomretryOverride());

        // sets boolean tolerances for auton refrence poses
        driveController.setTolerance(commander.getRefrenceTolerances());
    }

    @Override
    public void enabled(RobotCommander commander) {
        // commands drivetrain based on target drivemode
        if (commander.getDriveMode() == DriveMode.percent) {
            percentDrive(commander.getDrivePercentCommand());
        } else if (commander.getDriveMode() == DriveMode.stateDrive) {
            stateDrive(commander.getDriveState(), commander.getDriveRotationState());
        }

        if (commander.getBrakeCommand()) {
            setControl(brake);
        }

        if (commander.getPidgeonReset()) {
            seedFieldRelative(
                    new Pose2d(currentState.Pose.getX(), currentState.Pose.getY(), Rotation2d.fromRadians(0)));
        }

        if (commander.getAngleSnapCommand() != -1) {
            autoTurnControl(commander.getDrivePercentCommand(), commander.getAngleSnapCommand());
        }

        if (commander.getLockPoseCommand()) {
            /*
             * double goalX = snapPose.getX();
             * double goalY = snapPose.getY();
             * double robotX = robotState.getDrivePose().getX();
             * double robotY = robotState.getDrivePose().getY();
             * double diffX = robotX - goalX;
             * double diffY = robotY - goalY;
             * double hypot = Math.hypot(diffX, diffY);
             * double strafeX = diffY / hypot;
             * double strafeY = -(diffX / hypot);
             * double distX = diffX / hypot;
             * double distY = diffY / hypot;
             * double right = commander.getDrivePercentCommand()[0];
             * double up = commander.getDrivePercentCommand()[1];
             * double totalX = strafeX * right + distX * up;
             * double totalY = strafeY * right + distY * up;
             * Rotation2d rot = new Rotation2d(diffX, diffY);
             * Pose2d strafePose = new Pose2d(totalX, totalY, rot);
             */
            // aiden, NO BAD

            double goalX = snapPose.getX();
            double goalY = snapPose.getY();
            double robotX = robotState.getDrivePose().getX();
            double robotY = robotState.getDrivePose().getY();
            double diffX = robotX - goalX;
            double diffY = robotY - goalY;
            double hypot = Math.hypot(diffX, diffY);
            Rotation2d rot = new Rotation2d(diffX, diffY);
            double angle = rot.getDegrees() + 180;
            System.out.println("diffx" + diffX);
            System.out.println("diffy" + diffY);
            System.out.println("slope" + diffY/diffX);
            System.out.println("ang" + angle);
            autoTurnControl(commander.getDrivePercentCommand(), angle);
            driveType = true;
            // make robot face goal point
            // set to robot centric

        } else {
            driveType = true;
            // in case something else makes it false just add in a thing to instead of
            // directly false read it and set it to what it was
        }
    }

    @Override
    public void disabled() {
        setControl(brake);
    }

    @Override
    public void reset() {
    }

}
