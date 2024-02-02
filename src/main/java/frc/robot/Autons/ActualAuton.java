package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class ActualAuton extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        ring1,
        shoot1,
        ring2,
        shoot2,
        ring3,
        shoot3,
        end
    }

    public Step step;


    Pose2d a = new Pose2d(8.29, .75, Rotation2d.fromDegrees(0));
    Pose2d b = new Pose2d(5.78, 1.75, Rotation2d.fromDegrees(0));
    Pose2d c = new Pose2d(8.29, 2.45, Rotation2d.fromDegrees(0));
    Pose2d d = new Pose2d(5.78, 2, Rotation2d.fromDegrees(0));
    Pose2d e = new Pose2d(8.29, 4.11, Rotation2d.fromDegrees(0));
    //Pose2d f = new Pose2d(5.78, .75, Rotation2d.fromDegrees(0));
    Pose2d g = new Pose2d(3.5, .75, Rotation2d.fromDegrees(0));
    Pose2d h = new Pose2d(3.07, 4.11, Rotation2d.fromDegrees(0));
    
    boolean isShooting = true; 
    
    public ActualAuton(RobotState robotState) {
        super(robotState);
        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
                generateTrajectory(List.of(robotState.getDrivePose(), a));
                step = Step.ring1;
                break;
            
            case ring1:
                if (queuePath(List.of(robotState.getDrivePose(), b), true)) {
                    step = Step.shoot1;
               } else {
                   step = Step.ring1;
               }
                break; 
            
            case shoot1: 
                if (queuePath(List.of(robotState.getDrivePose(), c), true)) {
                    step = Step.ring2;
               } else {
                    step = Step.shoot1;
               }
                break; 

            case ring2:
                if (queuePath(List.of(robotState.getDrivePose(), d), true)) {
                    step = Step.shoot2;
               } else {
                    step = Step.ring2;
               }
                break;      
                
             case shoot2:
                if (queuePath(List.of(robotState.getDrivePose(), e), true)) {
                    step = Step.ring3;
               } else {
                    step = Step.shoot2;
               }
                break;   
            
                case ring3:
                if (queuePath(List.of(robotState.getDrivePose(), g), true)) {
                    step = Step.shoot3;
               } else {
                    step = Step.ring3;
               }
                break;    
                
                case shoot3:
                if (queuePath(List.of(robotState.getDrivePose(), h), true)) {
                    step = Step.end;
               } else {
                    step = Step.shoot3;
               }
                break;   
            
            case end:

            break; 
             
        }

        if (step != Step.end) {
            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            swerveBrake = true;
        }
        
        SmartDashboard.putString("Step", step.toString());

        // path visualizer, poses put to dashboard for advantagescope or glass
        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.start;
    }
}