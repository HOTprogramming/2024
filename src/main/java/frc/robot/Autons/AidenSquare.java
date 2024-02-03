package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.RobotState;


public class AidenSquare extends AutonBase {
    enum Step {
        toCorner1,
        toCorner2,
        toCorner3,
        toCorner4,
        end
    }

    public Step step;

    // made on blue side
    Pose2d Corner1 = new Pose2d(0, 3, Rotation2d.fromDegrees(0));
    Pose2d Corner2 = new Pose2d(3, 3, Rotation2d.fromDegrees(0));
    Pose2d Corner3 = new Pose2d(3, 0, Rotation2d.fromDegrees(0));
    Pose2d Corner4 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));


    public AidenSquare(RobotState robotState) {
        super(robotState);

        startPose = Corner4; //Pose2d(1.4, 1.7, Rotation2d.fromDegrees(0))
    }


    @Override
    public void runAuto() {


        switch (step) {
            case toCorner1:
                if (queuePath(List.of(robotState.getDrivePose(), Corner2), true)) {
                     step = Step.toCorner2;
                } else {
                    step = Step.toCorner1;
                }
            break;

            case toCorner2:
                if (queuePath(List.of(robotState.getDrivePose(), Corner3), true)) {
                     step = Step.toCorner3;
                } else {
                    step = Step.toCorner2;
                }
            break;

            case toCorner3:
                if (queuePath(List.of(robotState.getDrivePose(), Corner4), true)) {
                     step = Step.toCorner4;
                } else {
                    step = Step.toCorner3;
                }
            break;

            case toCorner4:
                if (queuePath(List.of(robotState.getDrivePose(), Corner1), true)) {
                     step = Step.end;
                } else {
                    step = Step.toCorner4;
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
        step = Step.toCorner1;
        generateTrajectory(List.of(robotState.getDrivePose(), Corner1));
    }
}