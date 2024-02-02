package frc.robot.Autons;

import static frc.robot.Constants.Auton.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class Blue3Park extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        move1,
        pew,
        ring1,
        shoot1,
        pewpew,
        ring2,
        shoot2,
        pewpewpew,
        ring3,
        shoot3,
        pewpewpewpew,
        park,
        end
    }

    public Step step;
    Pose2d start = new Pose2d(1.424, 3.161, Rotation2d.fromDegrees(0));
    Pose2d move1 = new Pose2d(3.785, 2.870, Rotation2d.fromDegrees(-40));
    Pose2d ring1 = new Pose2d(7.991, 0.759, Rotation2d.fromDegrees(0));
    Pose2d int1 = new Pose2d(7,2.588,Rotation2d.fromDegrees(-7));
    Pose2d shoot1 = new Pose2d(4.822, 4.682, Rotation2d.fromDegrees(-17));
    Pose2d ring2 = new Pose2d(7.991, 2.374, Rotation2d.fromDegrees(0));
    Pose2d shoot2 = new Pose2d(4.822, 4.682, Rotation2d.fromDegrees(-17));
    Pose2d ring3 = new Pose2d(7.991, 4.116, Rotation2d.fromDegrees(0));
    Pose2d shoot3 = new Pose2d(4.822, 4.682, Rotation2d.fromDegrees(-17));
    Pose2d int2 = new Pose2d(3.161, 4.922, Rotation2d.fromDegrees(-7));
    Pose2d park = new Pose2d(2.029, 4.762, Rotation2d.fromDegrees(0));
   

    public Blue3Park(RobotState robotState) {
        super(robotState);
        startPose = new Pose2d(1.424, 3.161, Rotation2d.fromDegrees(0));
    }
 

    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
                generateTrajectory(List.of(startPose, move1));
             
                step = Step.move1;
                break;

            case move1:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.pew;
                } else {
                    step = Step.move1;
                }
                break;

            case pew:
                if(queuePath(List.of(robotState.getDrivePose(), ring1), true)) {
                    step = Step.ring1;
                } else {
                    step = Step.pew;
                }
                break;

            case ring1:
                if(queuePath(List.of(robotState.getDrivePose(), int1, shoot1), true)) {
                    step = Step.shoot1;
                } else {
                    step = Step.ring1;
                }
                break;

            case shoot1:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.pewpew;
                } else {
                    step = Step.shoot1;
                }
                break;

            case pewpew:
                if(queuePath(List.of(robotState.getDrivePose(), ring2), true)) {
                    step = Step.ring2;
                } else {
                    step = Step.pewpew;
                }
                break;

            case ring2:
                if(queuePath(List.of(robotState.getDrivePose(), shoot2), true)) {
                    step = Step.shoot2;
                } else {
                    step = Step.ring2;
                }
                break;

            case shoot2:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.pewpewpew;
                } else {
                    step = Step.shoot2;
                }
                break;

            case pewpewpew:
                if(queuePath(List.of(robotState.getDrivePose(), ring3), true)) {
                    step = Step.ring3;
                } else {
                    step = Step.pewpewpew;
                }
                break;

            case ring3:
                if(queuePath(List.of(robotState.getDrivePose(), shoot3), true)) {
                    step = Step.shoot3;
                } else {
                    step = Step.ring3;
                }
                break;

            case shoot3:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.pewpewpewpew;
                } else {
                    step = Step.shoot3;
                }
                break;

            case pewpewpewpew:
                if(queuePath(List.of(robotState.getDrivePose(), int2, park), true)) {
                    step = Step.park;
                } else {
                    step = Step.pewpewpewpew;
                }
                break;

            case park:
                if(checkTime() || robotState.getAtTargetPose()) {
                    step = Step.end;
                } else {
                    step = Step.park;
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