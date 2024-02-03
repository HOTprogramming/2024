package frc.robot.Autons;

import java.util.List;

import com.fasterxml.jackson.core.StreamReadCapability;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;

public class Blue4Ring extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        ring1,
        shoot1,
        ring2,
        shoot2,
        ring3,
        shoot3,
        ring4,
        shoot4,
        end
    }

    public Step step;
    // pose 2D here of desired positions
    Pose2d ring1Pose2d = new Pose2d(8.29, 0.75, Rotation2d.fromDegrees(0));
    Pose2d ring2Pose2d = new Pose2d(8.29, 2.43, Rotation2d.fromDegrees(0));
    Pose2d ring3Pose2d = new Pose2d(8.29, 4.11, Rotation2d.fromDegrees(0));
    Pose2d ring4Pose2d = new Pose2d(8.29, 5.78, Rotation2d.fromDegrees(0));
    boolean holdingRing = false;
    boolean ringExists = true;

    public Blue4Ring(RobotState robotState) {
        super(robotState);
        // starting pos code here + any extra inits
    }

    @Override
    public void runAuto() {

        switch (step) {
            case start:
                generateTrajectory(List.of(startPose, ring1Pose2d));
                step = Step.ring1;
                break;
            case ring1:
                if (!ringExists) {
                    step = Step.ring2;
                }
                if (holdingRing || (queuePath(List.of(robotState.getDrivePose(), ring1Pose2d), true))) {
                    
                    holdingRing = false;
                    ringExists = true;
                    step = Step.ring2;
                }
                break;

            case shoot1:
                // shoot()
                generateTrajectory(List.of(ring1Pose2d, ring2Pose2d));
                holdingRing = false;
                ringExists = true;
                step = Step.ring2;
                break;

            case ring2:
                // generateTrajectory(List.of(startPose, endPose));

                if (!ringExists) {
                    step = Step.ring3;
                }
                if (holdingRing || (queuePath(List.of(robotState.getDrivePose(), ring2Pose2d), true))) {
                    
                    holdingRing = false;
                    ringExists = true;
                    step = Step.ring3;
                }
                break;

            case shoot2:
                // shoot()
                generateTrajectory(List.of(ring2Pose2d, ring3Pose2d));
                holdingRing = false;
                ringExists = true;
                step = Step.ring3;
                break;

            case ring3:
                // generateTrajectory(List.of(startPose, endPose));

                if (!ringExists) {
                    step = Step.ring4;
                }
                if (holdingRing || (queuePath(List.of(robotState.getDrivePose(), ring3Pose2d), true))) {
                    
                    holdingRing = false;
                    ringExists = true;
                    step = Step.ring4;
                }
                break;

            case shoot3:
                // shoot()
                generateTrajectory(List.of(ring3Pose2d, ring4Pose2d));
                holdingRing = false;
                ringExists = true;
                step = Step.ring4;
                break;

            case ring4:
                // generateTrajectory(List.of(startPose, endPose));

                if (!ringExists) {
                    step = Step.end;
                }
                if (holdingRing || (queuePath(List.of(robotState.getDrivePose(), ring4Pose2d), true))) {
                    holdingRing = false;
                    ringExists = false;
                    step = Step.end;
                }
                break;

            case shoot4:
                // shoot()

                holdingRing = false;
                ringExists = false;
                step = Step.end;
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