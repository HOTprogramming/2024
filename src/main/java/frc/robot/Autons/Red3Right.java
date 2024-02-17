package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;

public class Red3Right extends AutonBase {

    enum Step {
        start,
        shoot0,
        ring1,
        driveshoot1, 
        shoot1,
        ring2,
        driveshoot2, 
        shoot2,
        ring3,
        driveshoot3, 
        shoot3,
        end
    }

    public Step step;
    public Pose2d ring1 = new Pose2d(8.29, 7.46, Rotation2d.fromDegrees(180));
    public Pose2d almostring1 = new Pose2d(10.73, 7, Rotation2d.fromDegrees(180));
    public Pose2d betweenrings = new Pose2d(13.47, 6.4, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot1 = new Pose2d(10.74, 7.1, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(12, 6.4, Rotation2d.fromDegrees(180));
    public Pose2d almostring2 = new Pose2d(10.72, 6.7, Rotation2d.fromDegrees(180));
    public Pose2d ring2 = new Pose2d(8.29, 5.78, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot2 = new Pose2d(10.73, 6.8, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(12, 6.4, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(8.29, 4.11, Rotation2d.fromDegrees(180));
    public Pose2d ringalmost3 = new Pose2d(11.5, 4, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostshoot3 = new Pose2d(11.8, 4.10, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot3 = new Pose2d(11.7, 4.11, Rotation2d.fromDegrees(180));
    public Pose2d shoot3 = new Pose2d(12.4, 5.5, Rotation2d.fromDegrees(180));

    public Red3Right(RobotState robotState) {
        super(robotState);
        // startPose = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180));
        startPose = new Pose2d(15.15, 6.3, Rotation2d.fromDegrees(180));
        // startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    }

    @Override
    public void runAuto() {

        switch (step) {

            case start:
                generateTrajectory(2, 3, List.of(startPose, betweenrings, almostring1, ring1));
                step = Step.ring1;
                break;

            case ring1:
                if (queuePath(2, 3, List.of(robotState.getDrivePose(), almostshoot1, shoot1), true)) {
                    step = Step.shoot1;
                    //runIntake = false;
                    //runArm = true; 
                } else {
                    step = Step.ring1;
                    //runIntake = true;
                    //runArm = false; 
                }
                break;

            case driveshoot1:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.shoot1;
                    //runShooter = false;
                    //timer.reset();

                } else {
                    step = Step.driveshoot1;
                    //runShooter = true;
                }
                break;

            case shoot1:
                if(queuePath(2, 3, List.of(robotState.getDrivePose(), almostring2, ring2), true)){ 
                    step = Step.ring2; 
                }
                else{ 
                    if(timer.get() >= .2){
                        //runShooter = true; 
                    }

                    step = Step.shoot1; 
                }
                break; 

            case ring2:
                if (queuePath(2, 3, List.of(robotState.getDrivePose(), almostshoot2, shoot2), true)) {
                    step = Step.shoot2;
                   // runIntake = false;
                   // runArm = true; 
                } else {
                    step = Step.ring2;
                    // runIntake = true;
                    //runArm = false; 
                }
                break;

            case driveshoot2:

            break; 

            case shoot2:
                if (queuePath (2,3, List.of(robotState.getDrivePose(), ringalmost3, ring3), true)) {
                    step = Step.ring3;
                   // runShooter = false;
                } else {
                    step = Step.shoot2;
                   // runShooter = true;

                }
                break;

            case ring3:
                if (queuePath(2,1,List.of(robotState.getDrivePose(),  almostshoot3, shoot3), true)) {
                    step = Step.shoot3;
                   // runIntake = false;
                   // runArm = true; 
                } else {
                    step = Step.ring3;
                    // runIntake = true;
                    // runArm = false; 
                }
                break;
            
            case driveshoot3:
            break; 

            case shoot3:
                if (checkTime() || robotState.getAtTargetPose()) {
                    step = Step.end;
                    // runShooter = false;
                } else {
                    step = Step.shoot3;
                    // runShooter = true;
                }
                break;

            case end:
                runIntake = false;
                runShooter = false;
                runArm = false; 
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