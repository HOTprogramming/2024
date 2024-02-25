package frc.robot.Subsystems;


import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Camera.CameraPositions;
import frc.robot.RobotCommander.DriveMode;
import frc.robot.utils.trajectory.CustomHolonomicDriveController;
import frc.robot.utils.trajectory.RotationSequence;

import java.sql.Driver;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SwerveDrivetrain implements SubsystemBase {
    RobotState robotState;
    ConstantsBase.Drivetrain constants;
    

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
    private static final PIDController xController = new PIDController(9, 0.15, .5);
    private static final PIDController yController = new PIDController(8.5, 0.13, .45);
    private static final PIDController thetaController = new PIDController(15.5, 0.1, .5);

    private static final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            xController, yController, thetaController);

    // TEMP pose 2d for get angle snap command
    private Pose2d blueSpeaker = new Pose2d(0.93, 5.55, Rotation2d.fromDegrees(0));
    private Pose2d redSpeaker = new Pose2d(16.579, 5.548, Rotation2d.fromDegrees(180));

    Map<CameraPositions, Double> perviousTimeStamp = new EnumMap<>(CameraPositions.class);

    public Drivetrain(RobotState robotState) {

        // call swervedriveDrivetrain constructor (parent class)
        super(robotState.getConstants().getDriveTrainConstants().DRIVETRAIN_CONSTANTS,
                robotState.getConstants().getDriveTrainConstants().FRONT_LEFT_MODULE_CONSTANTS, 
                robotState.getConstants().getDriveTrainConstants().FRONT_RIGHT_MODULE_CONSTANTS, 
                robotState.getConstants().getDriveTrainConstants().BACK_LEFT_MODULE_CONSTANTS,
                robotState.getConstants().getDriveTrainConstants().BACK_RIGHT_MODULE_CONSTANTS);
                
        this.constants = robotState.getConstants().getDriveTrainConstants();

        // initalize robot state
        this.robotState = robotState;

        configNeutralMode(NeutralModeValue.Brake);
        fieldTypePublisher.set("Field2d");

    }

    /**
     * Controller method built for joystick output
     * 
     * @param drivePercents index of length 3, contains values -1 to 1
     * @param fieldCentricDrive drive fieldcentricly or robotcentricaly
     */
    private void percentDrive(double[] drivePercents, boolean fieldCentricDrive) {
        if (fieldCentricDrive) {
            setControl(fieldCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                    .withRotationalRate(drivePercents[2] * constants.MAX_ANGULAR_VELOCITY_RADS));
        } else {
            setControl(robotCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                    .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                    .withRotationalRate(drivePercents[2] * constants.MAX_ANGULAR_VELOCITY_RADS));
        }
    }

    /**
     * Turn automatically in teleop
     * 
     * @param drivePercents joystick commands
     * @param targetTheta target rotation to hit
     * @param fieldCentricDrive driving fieldcentricaly or robotcentricaly
     */
    private void autoTurnControl(double[] drivePercents, Rotation2d targetTheta, boolean fieldCentricDrive) {
        // Uses theta control to rotate (Rotation2d)
        if (fieldCentricDrive) {
            setControl(fieldCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                                    .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                                    .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                                            targetTheta.getRadians())));
        } else {
            setControl(robotCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                                    .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                                    .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                                            targetTheta.getRadians())));
        }

    }

    private void autoTurnControl(State holoDriveState, RotationSequence.State rotationState, Rotation2d targetTheta) {
        // Uses theta control to rotate (Rotation2d)
            ChassisSpeeds chassisSpeeds = driveController.calculate(getState().Pose, holoDriveState, rotationState);

            setControl(robotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                                    .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                                    .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                                            targetTheta.getRadians())));
        

    }

    private void stateDrive(State holoDriveState, RotationSequence.State rotationState) {
        SmartDashboard.putBoolean("Step_Actuallydriving", false);

        if (holoDriveState != null && rotationState != null) {
            SmartDashboard.putBoolean("Step_Actuallydriving", true);
            // drive with target states from trajectory generator (auton)
            ChassisSpeeds chassisSpeeds = driveController.calculate(currentState.Pose, holoDriveState, rotationState);
            setControl(withChassisSpeeds.withSpeeds(chassisSpeeds));
        }

    }

    /**
     * used to point the robot at a relative position
     * 
     * @param snapTranslation pose to point at usefull for intaking
     * @return target angle for intake side
     */
    private Rotation2d pointAt(Translation2d snapTranslation) {
        return new Rotation2d(snapTranslation.getX(), snapTranslation.getY());
    }

    /**
     * used to point the robot at an absolute pose
     * 
     * @param snapPose pose to point at. Usefull for shooting
     * @return target angle for shooter side
     */
    private Rotation2d pointAt(Pose2d snapPose) {
        return this.pointAt(currentState.Pose.minus(snapPose).getTranslation());
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
            if (robotState.getAlliance() == Alliance.Blue) {
                robotState.setPoseToSpeaker(currentState.Pose.minus(blueSpeaker).getTranslation().getNorm());
            }
            else{
                robotState.setPoseToSpeaker(currentState.Pose.minus(redSpeaker).getTranslation().getNorm());
            }

            
            robotState.setDrivePose(currentState.Pose);
            double currentTime = Utils.getCurrentTimeSeconds();
            double timeDifference = currentTime - lastTime;
            lastTime = currentTime;
            Translation2d distanceDifference = currentState.Pose.minus(lastPose).getTranslation();
            lastPose = currentState.Pose;

            velocities = distanceDifference.div(timeDifference);

            robotState.setDriveVelocity(velocities);

            // publishes pose to smartdashboard
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


        robotState.getVisionMeasurements().forEach((key,measurement) -> {
            if (measurement != null) {
                double fpgaTimeStamp = Timer.getFPGATimestamp();
                double imageTimeStamp = fpgaTimeStamp + (measurement.getTimestamp() - perviousTimeStamp.get(key));
                addVisionMeasurement(measurement.getPose(), imageTimeStamp, measurement.getSdtDeviation());
            
                perviousTimeStamp.put(key,measurement.getTimestamp());
            }
        });

        // updates module states for finding encoder offsets
        if (currentState.ModuleStates != null) {
            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumber("Swerve Encoder " + i + " (rads)",
                        currentState.ModuleStates[i].angle.getRadians());
            }
        }
        
    }

    Field2d desiredField = new Field2d();

    @Override
    public void init(RobotCommander commander) {
        // sets start pose to auton start pose
        seedFieldRelative(commander.getOdomretryOverride());

        // sets boolean tolerances for auton refrence poses
        driveController.setTolerance(commander.getRefrenceTolerances());

        SmartDashboard.putData("Desired Field", desiredField);
    }

    
    @Override
    public void enabled(RobotCommander commander) {
        // commands drivetrain based on target drivemode
        if (commander.getDriveMode() == DriveMode.percent) {
            percentDrive(commander.getDrivePercentCommand(), true);
        } else if (commander.getDriveMode() == DriveMode.stateDrive) {
            stateDrive(commander.getDriveState(), commander.getDriveRotationState());
        }
        
        try{
      desiredField.setRobotPose(new Pose2d(commander.getDriveState().poseMeters.getX(),
                                                commander.getDriveState().poseMeters.getY(),
                                                commander.getDriveRotationState().position));    
        } catch(Exception e){

        }

        if (commander.getBrakeCommand()) {
            setControl(brake);
        }


        if (commander.getPidgeonReset()) {
            m_pigeon2.setYaw(0);
            
        }

        if (commander.getAngleSnapCommand() != -1) {
            autoTurnControl(commander.getDrivePercentCommand(), Rotation2d.fromDegrees(commander.getAngleSnapCommand()), true);
        }

        if (commander.getLockSpeakerCommand()) {
            
            if (robotState.getAlliance() == Alliance.Red) {
                autoTurnControl(commander.getDrivePercentCommand(), pointAt(redSpeaker).plus(Rotation2d.fromDegrees(180)), true);
            } else {
                autoTurnControl(commander.getDrivePercentCommand(), pointAt(blueSpeaker), true);
            } 
        }

        // if (commander.getLockRingCommand()) {
        //     autoTurnControl(commander.getDrivePercentCommand(), pointAt(robotState.getVisionRingTranslation), true);
        // }

        if (commander.getResetRobotPose()) {
            // seedFieldRelative(new Pose2d(13.47, 4.11, Rotation2d.fromDegrees(0)));
            // if (robotState.getVisionTimestamps()[3] != -1) {
            //     seedFieldRelative(robotState.getVisionMeasurements()[3]);
            // }
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