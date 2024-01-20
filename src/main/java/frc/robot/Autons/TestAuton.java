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
    public Pose2d firstRingPose = new Pose2d(2, 5, Rotation2d.fromDegrees(0));
    public Pose2d firstShootPose = new Pose2d(3, 3, Rotation2d.fromDegrees(0));
    public Pose2d endPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    public TestAuton(RobotState robotState) {
        super(robotState);
        
        startPose = new Pose2d(2, 1, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        
        switch (step) {
            case firstRing:
                step = (queuePath(List.of(robotState.getDrivePose(), firstShootPose), true)) ? Step.firstShoot : Step.firstRing;
                break;
        
            case firstShoot:
                step = (queuePath(List.of(robotState.getDrivePose(), endPose), true)) ? Step.driveToEnd : Step.firstShoot;
                break;
                
            case driveToEnd:
                step = (queuePath(List.of(robotState.getDrivePose(), endPose), true)) ? Step.end : Step.driveToEnd;
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
        generateTrajectory(List.of(robotState.getDrivePose(), firstRingPose));
    }
}