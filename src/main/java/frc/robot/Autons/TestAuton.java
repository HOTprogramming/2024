package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.RobotState;


public class TestAuton extends AutonBase {
    enum Step {
        firstRing,
        firstShoot,
        driveToEnd,
        end
    }

    public Step step;

    // made on blue side
    Pose2d betweenRingPose = new Pose2d(5.78, 1.3, Rotation2d.fromDegrees(0));
    Pose2d firstRingPose = new Pose2d(8.29, .75, Rotation2d.fromDegrees(0));
    Pose2d betweenShootPose = new Pose2d(5.78, 2.8, Rotation2d.fromDegrees(0));
    Pose2d firstShootPose = new Pose2d(4, 2.25, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    public TestAuton(RobotState robotState) {
        super(robotState);
        
        startPose = new Pose2d(1.4, 1.7, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        
        switch (step) {
            case firstRing:
                if (queuePath(List.of(robotState.getDrivePose(), betweenShootPose, firstShootPose), true)) {
                     step = Step.firstShoot;
                } else {
                    step = Step.firstRing;
                }
               break;
        
            case firstShoot:
                if (queuePath(List.of(robotState.getDrivePose(), endPose), true)) {
                    step = Step.driveToEnd;
                } else {
                    step = Step.firstShoot;
                }
                break;
                
            case driveToEnd:
                if (checkTime()) {
                    step = Step.end;
                } else {
                    step = Step.driveToEnd;
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
        step = Step.firstRing;
        generateTrajectory(List.of(robotState.getDrivePose(), betweenRingPose, firstRingPose));
    }
}