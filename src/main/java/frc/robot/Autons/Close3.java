package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class Close3 extends AutonBase {
   
    enum Step {
        start,
        ring1,
        shoot1,
        ring2,
        shoot2,
        ring3,
        ringalmost3,
        shoot3,
        end
    }

    public Step step;
    public Pose2d ring1 = new Pose2d(13.47, 7.01, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(15, 6, Rotation2d.fromDegrees(180));
    public Pose2d ring2 = new Pose2d(13.47, 5.46, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(15, 4.7, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(14, 4.11, Rotation2d.fromDegrees(-120));
    public Pose2d shoot3 = new Pose2d(15, 2.5, Rotation2d.fromDegrees(90));

    public Close3(RobotState robotState) {
        super(robotState);
        //startPose = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180));
        startPose = new Pose2d(15, 5.465, Rotation2d.fromDegrees(180));
        //startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            
            case start:
            generateTrajectory(2, 3, List.of(startPose, ring1));
            step = Step.ring1;
            break;

            case ring1:
            if (queuePath(2, 3, List.of(robotState.getDrivePose(), shoot1), true)) {
                step = Step.shoot1;
                runIntake = false; 
                runArm = true; 
           } else {
               step = Step.ring1;
               runIntake = true; 
               runArm = false; 
           }
          break;

          case shoot1:
          if (queuePath(2, 3, List.of(robotState.getDrivePose(), ring2), true)) {
            step = Step.ring2;
            runShooter = false; 
       } else {
           step = Step.shoot1;
           runShooter = true; 
       }
      break;

         case ring2:
         if (queuePath(2, 3, List.of(robotState.getDrivePose(), shoot2), true)) {
            step = Step.shoot2;
            runIntake = false;
            runArm = true; 
       } else {
           step = Step.ring2;
           runIntake = true;
           runArm = false; 
       }
      break;

       case shoot2:
        if (queuePath(2, 3, List.of(robotState.getDrivePose(), ring3), true)) {
            step = Step.ring3;
            runShooter = false;
       } else {
           step = Step.shoot2;
           runShooter = true;
       }
       break;

       case ring3:
        if (queuePath(2, 3, List.of(robotState.getDrivePose(), shoot3), true)) {
            step = Step.end;
            runIntake = false;
            runArm = true; 
       } else {
           step = Step.ring3;
           runIntake = true;
           runArm = false; 
       }
       break;

       case shoot3:
        if (checkTime() || robotState.getAtTargetPose()) {
            step = Step.end;
            runShooter = false;
       } else {
           step = Step.shoot3;
           runShooter = true;
       }
       break;

        case end:
        runIntake = false; 
        runShooter = false; 
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