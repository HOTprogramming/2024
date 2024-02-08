package frc.robot.Autons;

import java.util.List;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class Blue3Under extends AutonBase {
    enum Step {
        start,
        Ring1,
        Shoot1,
        Ring2,
        Shoot2,
        Ring3,
        Shoot3,
        driveToEnd,
        end
    }

    public Step step;

    // made on blue side
    Pose2d betweenRing1Pose = new Pose2d(5.78, 1.3, Rotation2d.fromDegrees(0));
    Pose2d Ring1Pose = new Pose2d(8.29, .75, Rotation2d.fromDegrees(-10));
    Pose2d betweenShoot1Pose = new Pose2d(5.78, 1.5, Rotation2d.fromDegrees(-20));
    Pose2d Shoot1Pose = new Pose2d(4, 2.25, Rotation2d.fromDegrees(-30));
    Pose2d betweenRing2Pose = new Pose2d(5.78, 1.5, Rotation2d.fromDegrees(-5));
    Pose2d Ring2Pose = new Pose2d(8.29, 2.43, Rotation2d.fromDegrees(20));
    Pose2d betweenShoot2Pose = new Pose2d(5.78, 1.5, Rotation2d.fromDegrees(0));
    Pose2d Shoot2Pose = new Pose2d(4, 2.25, Rotation2d.fromDegrees(-30));
    Pose2d betweenRing3Pose = new Pose2d(5, 4, Rotation2d.fromDegrees(0));
    Pose2d Ring3Pose = new Pose2d(8.29, 4.11, Rotation2d.fromDegrees(0));
    Pose2d betweenShoot3Pose = new Pose2d(4.5, 3.7, Rotation2d.fromDegrees(-15));
    Pose2d Shoot3Pose = new Pose2d(4, 2.25, Rotation2d.fromDegrees(-30));
    Pose2d endPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    public Blue3Under(RobotState robotState) {
        super(robotState);
        
        // refrenceTolerances = new Pose2d(.2, .2, Rotation2d.fromDegrees(5));
        startPose = new Pose2d(1.4, 1.7, Rotation2d.fromDegrees(0));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
                generateTrajectory(List.of(startPose, betweenRing1Pose, Ring1Pose));
                step = Step.Ring1;
                
                break;
            case Ring1:
                if (queuePath(List.of(robotState.getDrivePose(), betweenShoot1Pose, Shoot1Pose), true)) {
                    step = Step.Shoot1;
                }
                break;
        
            case Shoot1:
                if (queuePath(List.of(robotState.getDrivePose(), betweenRing2Pose, Ring2Pose), true)) {
                    step = Step.Ring2;
                }
                break;
            
            case Ring2:
                if (queuePath(constants.AUTON_DEFAULT_MAX_VELOCITY_METERS, 1, 0, 0, List.of(robotState.getDrivePose(), betweenShoot2Pose, Shoot2Pose, betweenRing3Pose, Ring3Pose), true)) {
                    step = Step.Ring3;
                }

            // case Ring2:
            //     if (queuePath(AUTON_DEFAULT_MAX_VELOCITY_METERS, AUTON_DEFAULT_MAX_ACCEL_METERS, 0, 2, List.of(robotState.getDrivePose(), betweenShoot2Pose, Shoot2Pose), true)) {
            //         step = Step.Shoot2;
            //     }
            //     break;
            
            // case Shoot2:
            //     if (queuePath(AUTON_DEFAULT_MAX_VELOCITY_METERS, AUTON_DEFAULT_MAX_ACCEL_METERS, 2, 0, List.of(robotState.getDrivePose(), betweenRing3Pose, Ring3Pose), true)) {
            //         step = Step.Ring3;
            //     }
            //     break;

            case Ring3:
                if (queuePath(List.of(robotState.getDrivePose(), betweenShoot3Pose, Shoot3Pose), true)) {
                    step = Step.Shoot3;
                }
                break;

            case Shoot3:
            if (queuePath(List.of(robotState.getDrivePose(), endPose), true)) {
                step = Step.driveToEnd;
            }
            break;
                
            case driveToEnd:
                if (checkTime() || robotState.getAtTargetPose())  {
                    step = Step.end;
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
        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.start;
    }
}