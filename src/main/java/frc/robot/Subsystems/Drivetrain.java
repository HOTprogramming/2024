package frc.robot.Subsystems;


import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.Autons.Center4Note;
import frc.robot.Autons.Center4NoteBlue;
import frc.robot.Autons.Right4NoteBlue;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Camera.CameraPositions;
import frc.robot.RobotCommander.DriveMode;
import frc.robot.utils.trajectory.CustomHolonomicDriveController;
import frc.robot.utils.trajectory.RotationSequence;
import edu.wpi.first.math.util.Units;


import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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

    DoubleArrayPublisher posePublisherKeith = table.getDoubleArrayTopic("RobotPoseKeith").publish();
    DoubleArrayPublisher velocityPublisherKieth = table.getDoubleArrayTopic("RobotVelocityKeith").publish();
    Rotation2d cachedRotation = Rotation2d.fromDegrees(0);



    // for velocity calcs
    private SwerveDriveState currentState;
    private Pose2d lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();
    private Translation2d velocities = new Translation2d(0, Rotation2d.fromDegrees(0));

    // Drive controllers
    private static final PIDController xController = new PIDController(10, 0, 0); // 9 .15 .5
    private static final PIDController yController = new PIDController(10, 0, 0); // 8.5 .13 .45
    private static final PIDController thetaController = new PIDController(10, 0, 0);
    
    private static final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            xController, yController, thetaController);

    // TEMP pose 2d for get angle snap command
    private Pose2d blueSpeaker = new Pose2d(0.1, 5.55, Rotation2d.fromDegrees(0));
    private Pose2d redSpeaker = new Pose2d(16.579, 5.688, Rotation2d.fromDegrees(180));

    private Pose2d blueLob = new Pose2d(0.1, 6.5, Rotation2d.fromDegrees(0));
    private Pose2d redLob = new Pose2d(16.579, 6.5, Rotation2d.fromDegrees(180));

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


        // this.m_odometry.
    }

    /**
     * Controller method built for joystick output
     * 
     * @param drivePercents index of length 3, contains values -1 to 1
     * @param fieldCentricDrive drive fieldcentricly or robotcentricaly
     */
    private void percentDrive(double[] drivePercents, boolean fieldCentricDrive) {
        
        // if (Math.abs(drivePercents[2]) > 0.01) {
            // cachedRotation = currentState.Pose.getRotation();

            if (fieldCentricDrive) {
                setControl(fieldCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                        .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                        .withRotationalRate(drivePercents[2] * constants.MAX_ANGULAR_VELOCITY_RADS));
            } else {
                setControl(robotCentric.withVelocityX(drivePercents[0] * constants.MAX_VELOCITY_METERS)
                        .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                        .withRotationalRate(drivePercents[2] * constants.MAX_ANGULAR_VELOCITY_RADS));
            }
        // } else {
            // autoTurnControl(drivePercents, cachedRotation, fieldCentricDrive);
        // }
        
        SmartDashboard.putNumber("CachedRotation", cachedRotation.getDegrees());
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

    private void autoXControl(double[] drivePercents, double xPose, Rotation2d targetTheta) {
        SmartDashboard.putBoolean("xatSet", xController.atSetpoint());
        SmartDashboard.putBoolean("tatSet", thetaController.atSetpoint());


        if (!xController.atSetpoint() || !thetaController.atSetpoint()) {
            setControl(fieldCentric.withVelocityX(xController.calculate(currentState.Pose.getX(), xPose))
                                .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                                .withRotationalRate(thetaController.calculate(currentState.Pose.getRotation().getRadians(),
                                            targetTheta.getRadians())));
        } else {
            thetaController.calculate(currentState.Pose.getRotation().getRadians());
            xController.calculate(currentState.Pose.getX());

            setControl(fieldCentric.withVelocityX(0)
                                    .withVelocityY(drivePercents[1] * constants.MAX_VELOCITY_METERS)
                                    .withRotationalRate(0));
        }
        
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

            posePublisherKeith.set(new double[] {
                    Units.metersToFeet(currentState.Pose.getX()),
                    Units.metersToFeet(currentState.Pose.getY()),
                    currentState.Pose.getRotation().getDegrees()
            });

            // publishes velocity to smartdashboard
            velocityPublisherKieth.set(new double[] {
                    Units.metersToFeet(velocities.getX()),
                    Units.metersToFeet(velocities.getY()),
                    velocities.getAngle().getDegrees()
            });
        }



        robotState.getVisionMeasurements().forEach((key,measurement) -> {
            measurement.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = robotState.getCameraStdDeviations().get(key);

                    addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
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

        if(commander.getAuto() != null){
            seedFieldRelative(commander.getAuto().startPose); //15.15);
        }

        // seedFieldRelative(new Pose2d(15.3, 6.7, Rotation2d.fromDegrees(155))); //15.15
         //15.15);

        // sets boolean tolerances for auton refrence poses
        xController.setTolerance(.01);
        thetaController.setTolerance(Units.degreesToRadians(2));

        SmartDashboard.putData("Desired Field", desiredField);
    }

    
    @Override
    public void teleop(RobotCommander commander) {
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
            cachedRotation = currentState.Pose.getRotation();

        }

        // if (commander.getLockParallel()) {
        //     cachedRotation = currentState.Pose.getRotation();

        //     if (robotState.getAlliance() == Alliance.Red) {
        //         autoTurnControl(commander.getDrivePercentCommand(), Rotation2d.fromDegrees(180), true);

        //     } else {
        //         autoTurnControl(commander.getDrivePercentCommand(), Rotation2d.fromDegrees(0), true);

        //     }
        // }

        // if (commander.getAngleSnapCommand() != -1) {
        //     cachedRotation = currentState.Pose.getRotation();

        //     autoTurnControl(commander.getDrivePercentCommand(), Rotation2d.fromDegrees(commander.getAngleSnapCommand()), true);
        // }

        if (commander.getLockAmpCommand()) {
            cachedRotation = currentState.Pose.getRotation();
            if (robotState.getAlliance() == Alliance.Red) {
                autoXControl(commander.getDrivePercentCommand(), 14.7, Rotation2d.fromDegrees(-90));
            } else {
                autoXControl(commander.getDrivePercentCommand(), 1.84, Rotation2d.fromDegrees(-90));
            } 
        }

        if (commander.getLockSpeakerCommand()) {
            cachedRotation = currentState.Pose.getRotation();
            if (robotState.getAlliance() == Alliance.Red) {
                if (robotState.getDrivePose().getX() < 8) {
                    autoTurnControl(commander.getDrivePercentCommand(), pointAt(redSpeaker).plus(Rotation2d.fromDegrees(180)), true);

                } else {
                    autoTurnControl(commander.getDrivePercentCommand(), pointAt(redLob).plus(Rotation2d.fromDegrees(180)), true);
                }
            } else {
                if (robotState.getDrivePose().getX() > 8) {
                    autoTurnControl(commander.getDrivePercentCommand(), pointAt(blueSpeaker), true);

                } else {
                    autoTurnControl(commander.getDrivePercentCommand(), pointAt(blueLob), true);
                }
            } 
        }


        // if (commander.getLockRingCommand()) {
        //     autoTurnControl(commander.getDrivePercentCommand(), pointAt(robotState.getVisionRingTranslation), true);
        // }

        if (commander.getResetRobotPose()) {
            // seedFieldRelative(new Pose2d(13.47, 4.11, Rotation2d.fromDegrees(0)));
            Optional<EstimatedRobotPose> measurment = robotState.getVisionMeasurements().get(CameraPositions.BACK);
            if (measurment.isPresent()) {
                seedFieldRelative(measurment.get().estimatedPose.toPose2d());
            }
        }
    }

    @Override
    public void cameraLights() {
        setControl(brake);
    }

    @Override
    public void reset() {
        cachedRotation = currentState.Pose.getRotation();
    }

    public void teleLimits(){
        for (int i = 0; i < ModuleCount; i++) {
            Modules[i].getDriveMotor().getConfigurator().apply(constants.TELEOP_SWERVE_DRIVE_GAINS);
            Modules[i].getSteerMotor().getConfigurator().apply(constants.TELEOP_SWERVE_STEER_GAINS);

            Modules[i].getDriveMotor().getConfigurator().apply(constants.TELEOP_DRIVE_CURRENT);
            Modules[i].getSteerMotor().getConfigurator().apply(constants.TELEOP_STEER_CURRENT);
        }
    }

    public void autoLimits(){
        for (int i = 0; i < ModuleCount; i++) {
            Modules[i].getDriveMotor().getConfigurator().apply(constants.SWERVE_DRIVE_GAINS);
            Modules[i].getSteerMotor().getConfigurator().apply(constants.SWERVE_STEER_GAINS);

            Modules[i].getDriveMotor().getConfigurator().apply(constants.AUTON_DRIVE_CURRENT);
            Modules[i].getSteerMotor().getConfigurator().apply(constants.AUTON_STEER_CURRENT);
        }
    }

}