package frc.robot.Autons;

import static frc.robot.Constants.Auton.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
// import frc.robot.Subsystems.Arm.armDesiredPos;


public class Red3Ring extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        shoot1,
        firstring1, 
        shoot2,
        actualshoot2,
        secondring1,
        shoot3,
        actualshoot3,
        end
    }

    public Step step;
    Pose2d firstring1 = new Pose2d(13.689, 6.283, Rotation2d.fromDegrees(180));
    Pose2d firstring2 = new Pose2d(8.503, 7.441, Rotation2d.fromDegrees(180));
    Pose2d shoot2 = new Pose2d(10.642, 6.384, Rotation2d.fromDegrees(170));
    Pose2d secondring1 = new Pose2d(8.390, 5.801, Rotation2d.fromDegrees(180));
    Pose2d shoot3 = new Pose2d(10.642, 6.384, Rotation2d.fromDegrees(170));
   

    public Red3Ring(RobotState robotState) {
        super(robotState);

        
        // startPose = new Pose2d(15.3, 5.465, Rotation2d.fromDegrees(180));
    }


    @Override
    public void runAuto() {
        
        switch (step) {
            case start:
            
            generateTrajectory(List.of(startPose, firstring1, firstring2));
            runArm = true;
            step = Step.firstring1;
            
            break;

            case firstring1:
            if (queuePath(List.of(robotState.getDrivePose(), shoot2), true) ) {
                step = Step.shoot2;
           } else {
               step = Step.firstring1;
               
           }
            break;


            case shoot2:
            if (checkTime() || robotState.getAtTargetPose()) {
                step = Step.actualshoot2;
           } else {
               step = Step.shoot2;
            //    desiredArmPos = armDesiredPos.shoot;
           }
           break; 

            case actualshoot2:
             if (queuePath(List.of(robotState.getDrivePose(), secondring1), true)) {
                step = Step.secondring1;
                
           } else {
               step = Step.actualshoot2;
           }
            break;

            case secondring1:
            if (queuePath(List.of(robotState.getDrivePose(), shoot3), true)) {
                step = Step.shoot3;
           } else {
               step = Step.secondring1;
           }
            break;

            case shoot3:
            if (checkTime() || robotState.getAtTargetPose()) {
                step = Step.actualshoot3;
           } else {
               step = Step.shoot3;
                // desiredArmPos = armDesiredPos.zero;
           }
           break; 

            case actualshoot3:
             
            step = Step.end;
    
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

        // path visualizer, poses put to dashboard for advantagescope or glass
        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.start;

        startPose = new Pose2d(15.3, 5.465, Rotation2d.fromDegrees(180));
    }
}