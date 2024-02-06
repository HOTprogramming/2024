package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;


public class Red2Left extends AutonBase {
   
    enum Step {
        start,
        ring1,
        shoot1,
        ring2,
        shoot2,
        end
    }

    public Step step;
    public Pose2d ring1 = new Pose2d(8.29, .75, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(13.5, 2.5, Rotation2d.fromDegrees(180));
    public Pose2d ring2 = new Pose2d(8.29, 2.43, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(13.5, 2.5, Rotation2d.fromDegrees(180));
   

    public Red2Left(RobotState robotState) {
        super(robotState);
        //startPose = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180));
        //startPose = new Pose2d(15.2, 5.6, Rotation2d.fromDegrees(180));
        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            
            case start:
            generateTrajectory(List.of(startPose, ring1));
            step = Step.ring1;
            break;

            case ring1:
            if (queuePath(List.of(robotState.getDrivePose(), shoot1), true)) {
                step = Step.shoot1;
           } else {
               step = Step.ring1;
           }
          break;

          case shoot1:
          if (queuePath(List.of(robotState.getDrivePose(), ring2), true)) {
            step = Step.ring2;
       } else {
           step = Step.shoot1;
       }
      break;

         case ring2:
         if (queuePath(List.of(robotState.getDrivePose(), shoot2), true)) {
            step = Step.shoot2;
       } else {
           step = Step.ring2;
       }
      break;

       case shoot2:
       step = step.end;
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