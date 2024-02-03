package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class Triangle extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        side1,
        side2,
        side3,
        end
    }

    public Step step;

    Pose2d corner1 = new Pose2d(2, 2, Rotation2d.fromDegrees(60));
    Pose2d corner2 = new Pose2d(4, 0, Rotation2d.fromDegrees(60));
    Pose2d corner3 = new Pose2d(0, 0, Rotation2d.fromDegrees(60));
    

    public Triangle(RobotState robotState) {
        super(robotState);
        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        // starting pos code here + any extra inits
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
                // generateTrajectory(List.of(startPose, endPose));
                generateTrajectory(List.of(robotState.getDrivePose(), corner1));
                step = Step.side1;
                break;
            
            case side1:
                if (queuePath(List.of(robotState.getDrivePose(), corner2), true)) {
                    step = Step.side2;
               } else {
                   step = Step.side1;
               }
                break; 
            
            case side2: 
                if (queuePath(List.of(robotState.getDrivePose(), corner3), true)) {
                    step = Step.side3;
               } else {
                    step = Step.side2;
               }
                break; 

            case side3:
                if (queuePath(List.of(robotState.getDrivePose(), corner2), true)) {
                    step = Step.end;
               } else {
                    step = Step.side3;
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