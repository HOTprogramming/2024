package frc.robot.Autons;

import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.RotationSequence;
import frc.robot.trajectory.Waypoint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AutonBase {
    ConstantsBase.Auton constants;
    Timer timer = new Timer();
    
    RobotState robotState;

    // default values if possible
    public Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public State holoDriveState = new State(0, 0, 0, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 0);
    public boolean swerveBrake = false;
    public Pose2d refrenceTolerances = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public RotationSequence.State rotationState = new RotationSequence.State(Rotation2d.fromDegrees(0), 0);

    
    TrajectoryConfig trajectoryConfig;
    CustomTrajectoryGenerator trajectoryGenerator;
    public boolean runShooter;
    public double driveSpeed;
    public double armSpeed;
    public boolean runArm;


    public AutonBase(RobotState robotState){
        this.robotState = robotState;

        constants = robotState.getConstants().getAutonConstants();
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
        trajectoryGenerator = new CustomTrajectoryGenerator();
        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
        trajectoryGenerator.generate(trajectoryConfig, waypoints);
    }

    public void generateTrajectory(List<Pose2d> points) {
        trajectoryConfig = new  TrajectoryConfig(constants.AUTON_DEFAULT_MAX_VELOCITY_METERS, constants.AUTON_DEFAULT_MAX_ACCEL_METERS);
        trajectoryGenerator = new CustomTrajectoryGenerator();
        
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
        if (trajectoryGenerator == null) {
            generateTrajectory(maxV, maxAccel, startV, endV, points);
            return true;
        }
        if ((!timed && robotState.getAtTargetPose()) || (timed && checkTime())) {
            timer.reset();

            generateTrajectory(maxV, maxAccel, startV, endV, points);
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
