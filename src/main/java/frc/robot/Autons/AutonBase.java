package frc.robot.Autons;

import frc.robot.RobotState;

import static frc.robot.Constants.Drivetrain.*;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.RotationSequence;
import frc.robot.trajectory.Waypoint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

public abstract class AutonBase {
    Timer timer = new Timer();
    
    RobotState robotState;

    public Pose2d startPose;
    public Boolean runShooter;
    public double driveSpeed;
    public State holoDriveState;
    public boolean swerveBrake;
    public Pose2d refrenceTolerances;
    public RotationSequence.State rotationState;
    
    TrajectoryConfig trajectoryConfig;
    CustomTrajectoryGenerator trajectoryGenerator;


    public AutonBase(RobotState robotState){
        this.robotState = robotState;
    }

    public void generateTrajectory(double maxV, double maxAccel, double startV, double endV, List<Pose2d> points) {
        trajectoryConfig = new  TrajectoryConfig(maxV, maxAccel).setStartVelocity(startV).setEndVelocity(endV);
        trajectoryGenerator = new CustomTrajectoryGenerator();
        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
    }

    public void generateTrajectory(List<Pose2d> points) {
        trajectoryConfig = new  TrajectoryConfig(AUTON_DEFAULT_MAX_VELOCITY_METERS, AUTON_DEFAULT_MAX_VELOCITY_METERS);
        trajectoryGenerator = new CustomTrajectoryGenerator();
        
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Pose2d point:points) {
            waypoints.add(Waypoint.fromHolonomicPose(point));
        }
    }


    public boolean checkTime() {
        return timer.get() >= trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds();
    }

    /**
     * best used with inline if statements (variable = (condition) ? expressionTrue :  expressionFalse;) 
     * 
     * @return if the new path was queued
     */
    public boolean queuePath(double maxV, double maxAccel, double startV, double endV, List<Pose2d> points, boolean timed) {
        if (trajectoryGenerator == null) {
            generateTrajectory(maxV, maxAccel, startV, endV, points);
        }
        if (robotState.getAtTargetPose() || (timed && checkTime())) {
            timer.reset();

            generateTrajectory(maxV, maxAccel, startV, endV, points);
            return true;
        } else {
            return false;
        }
    }

     /**
     * best used with inline if statements (variable = (condition) ? expressionTrue :  expressionFalse;) 
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
