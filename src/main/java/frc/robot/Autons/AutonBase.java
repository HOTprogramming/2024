package frc.robot.Autons;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.Subsystems.Arm.ArmCommanded;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.trajectory.CustomTrajectoryGenerator;
import frc.robot.utils.trajectory.RotationSequence;
import frc.robot.utils.trajectory.Waypoint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AutonBase {
    ConstantsBase.Auton constants;
    Timer timer = new Timer();
    
    RobotState robotState;

    private Pose2d blueSpeaker = new Pose2d(0.1, 5.55, Rotation2d.fromDegrees(0));
    private Pose2d redSpeaker = new Pose2d(16.579, 5.688, Rotation2d.fromDegrees(180));

    // default values if possible
    public Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public State holoDriveState = new State(0, 0, 0, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 0);
    public boolean swerveBrake = false;
    public Pose2d refrenceTolerances = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(15));
    public RotationSequence.State rotationState = new RotationSequence.State(Rotation2d.fromDegrees(0), 0);

    
    TrajectoryConfig trajectoryConfig;
    CustomTrajectoryGenerator trajectoryGenerator = new CustomTrajectoryGenerator();
    public boolean runShooter = false;
    public boolean runIntake = false; 
    public ArmCommanded armCommand = ArmCommanded.zero; 
    public boolean seedPose = false;
    public boolean runFeeder = false;
    public boolean autoAim = false;
    public boolean zeroArm = false;
    public boolean driving = false;
    public boolean unPackage = false;
    


    public AutonBase(RobotState robotState){
        this.robotState = robotState;
    

        constants = robotState.getConstants().getAutonConstants();
    }

    public double calculateArmHint(Pose2d shootPose) {
        if (robotState.getAlliance() == Alliance.Blue) {
            return shootPose.minus(blueSpeaker).getTranslation().getNorm();
        } else {
            return shootPose.minus(redSpeaker).getTranslation().getNorm();
        }
    }

    public void visualizePath() {
        Pose2d idealPose;

        if (trajectoryGenerator != null) {
            idealPose = trajectoryGenerator.getDriveTrajectory().sample(timer.get()).poseMeters;
        } else {
            idealPose = startPose;
        }
        SmartDashboard.putNumberArray("Trajectory", new Double[] {idealPose.getX(), 
                                                                    idealPose.getY(), 
                                                                    idealPose.getRotation().getDegrees()});
    }

    public void generateTrajectory(double maxV, double maxAccel, double startV, double endV, List<Pose2d> points) {
        trajectoryConfig = new  TrajectoryConfig(maxV, maxAccel).setStartVelocity(startV).setEndVelocity(endV);
        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
        trajectoryGenerator.generate(trajectoryConfig, waypoints);
    }

    public void generateTrajectory(List<Pose2d> points) {
        trajectoryConfig = new  TrajectoryConfig(constants.AUTON_DEFAULT_MAX_VELOCITY_METERS, constants.AUTON_DEFAULT_MAX_ACCEL_METERS);
        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
        trajectoryGenerator.generate(trajectoryConfig, waypoints);
    }

    public void generateTrajectory(double maxV, double maxAccel, List<Pose2d> points) {
        timer.reset();
        trajectoryConfig = new  TrajectoryConfig(maxV, maxAccel);

        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
        trajectoryGenerator.generate(trajectoryConfig, waypoints);
    }

    /**
     * checks if you should be at the target pose
     * 
     * @return is trajectory time over
     */
    public boolean checkTime() {
        return timer.get() >= trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds();
    }

    /**
     * call to queue a path, and know if the current path is complete.
     * 
     * @return if the new path was queued
     */
    public boolean queuePath(double maxV, double maxAccel, double startV, double endV, List<Pose2d> points, boolean timed) {
        if ((!timed && robotState.getAtTargetPose()) || (timed && checkTime())) {
            timer.reset();

            generateTrajectory(maxV, maxAccel, startV, endV, points);
            return true;
        } else {
            return false;
        }
    }

    public boolean queuePath(double maxV, double maxAccel, List<Pose2d> points, boolean timed) {
        if ((!timed && robotState.getAtTargetPose()) || (timed && checkTime())) {
            timer.reset();

            generateTrajectory(maxV, maxAccel, points);
            return true;
        } else {
            return false;
        }
    }
     /**
     * call to queue a path, and know if the current path is complete.
     * uses default trajectory configs
     * 
     * @return if the new path was  queued
     */
    public boolean queuePath(List<Pose2d> points, boolean timed) {
        if (trajectoryGenerator == null) {
            generateTrajectory(points);
        }
        if (robotState.getAtTargetPose() || (timed && checkTime())) {
            timer.reset();

            generateTrajectory(points);
            return true;
        } else {
            return false;
        }
    }

    public abstract void runAuto();
    
    public void reset() {
        timer.start();
        timer.reset();
    }
}
