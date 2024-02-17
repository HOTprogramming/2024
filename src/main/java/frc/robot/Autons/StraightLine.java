package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class StraightLine extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        driving,
        end
    }

    public Step step;
    // pose 2D here of desired positions
   

    public StraightLine(RobotState robotState) {
        super(robotState);
        // starting pos code here + any extra inits
        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
                generateTrajectory(List.of(robotState.getDrivePose(), new Pose2d(3, 0, Rotation2d.fromDegrees(0))));
             
                step = Step.driving;
                break;
            
            case driving:
                if (checkTime()) {
                    step = Step.end;
                }

                break; 

            case end:
                // end motor control

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