package frc.robot.Autons;

import static frc.robot.Constants.Auton.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;

public class Red2Ring extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        tonote1approach2,
        tonote1approach1,
        tonote1,
        toshoot1,
        tonote2approach2,
        tonote2approach1,
        tonote2,
        toshoot2,
        end
    }

    public Step step;
    // pose 2D here of desired positions
    public Pose2d note1 = new Pose2d(13.47, 7.01, Rotation2d.fromDegrees(180));
    public Pose2d note1approach1 = new Pose2d(13.97, 7.01, Rotation2d.fromDegrees(180));
    public Pose2d note1approach2 = new Pose2d(14.47, 7.01, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(14.5, 4.5, Rotation2d.fromDegrees(30));
    public Pose2d note2 = new Pose2d(13.47, 5.56, Rotation2d.fromDegrees(180));
    public Pose2d note2approach1 = new Pose2d(13.97, 5.56, Rotation2d.fromDegrees(180));
    public Pose2d note2approach2 = new Pose2d(14.47, 5.56, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(14.5, 4.5, Rotation2d.fromDegrees(30));

    public Red2Ring(RobotState robotState) {
        super(robotState);
        // starting pos code here + any extra inits
        startPose = new Pose2d(15.2, 5.6, Rotation2d.fromDegrees(180));
    }

    @Override
    public void runAuto() {

        switch (step) {
            case start:
                // generateTrajectory(List.of(startPose, endPose));
                generateTrajectory(List.of(startPose, note1approach2, note1approach1, note1));
                step = Step.tonote1;
                break;
/*
            case tonote1approach2:
                step = Step.tonote1approach1;
                break;

            case tonote1approach1:
                step = Step.tonote1;
                break;
*/
            case tonote1:
                if (queuePath(List.of(robotState.getDrivePose(), note1approach2, note1approach1, note1), true)) {
                    step = Step.toshoot1;
                }
                break;

            case toshoot1:
                if (queuePath(List.of(robotState.getDrivePose(), shoot1), true)) {
                    step = Step.tonote2;
                }
                
                break;
/*
            case tonote2approach2:
                step = Step.tonote2approach1;
                break;

            case tonote2approach1:
                step = Step.tonote2;
                break;
*/
            case tonote2:
                if (queuePath(List.of(robotState.getDrivePose(), note2approach2, note2approach1, note2), true)) {
                    step = Step.toshoot2;
                }
                break;

            case toshoot2:
                if (queuePath(List.of(robotState.getDrivePose(), shoot2), true)) {
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