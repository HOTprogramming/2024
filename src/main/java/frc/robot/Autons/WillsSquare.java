package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.RobotState;


public class WillsSquare extends AutonBase {
    enum Step {
        side1,
        side2,
        side3,
        end
    }

    public Step step;

    // made on blue side
    Pose2d corner1 = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
    Pose2d corner2 = new Pose2d(1, 1, Rotation2d.fromDegrees(-90));
    Pose2d corner3 = new Pose2d(0, 1, Rotation2d.fromDegrees(-180));
    Pose2d corner0 = new Pose2d(0, 0, Rotation2d.fromDegrees(-270)); // Thanks Aiden :)

    public WillsSquare(RobotState robotState) {
        super(robotState);
        
        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        
        switch (step) {
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
                if (queuePath(List.of(robotState.getDrivePose(), corner0), true)) {
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
    }

    @Override
    public void reset() {
        super.reset();
        step = Step.side1;
        generateTrajectory(List.of(robotState.getDrivePose(), corner1));
    }
}
