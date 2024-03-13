package frc.robot.Autons;


import java.util.List;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.Waypoint;

//red auton
public class BlueOppositeAmp extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        ring1,
        shot1,
        shootBeforeRing2,
        ring2,
        shot2,
        shootBeforeRing3,
        ring3,
        shot3,
        end;
    }

    public Step step = Step.start;   

    public BlueOppositeAmp(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(1.45, 4.63, Rotation2d.fromDegrees(-70)); //15.15

        seedPose = true;
    }
    
    Pose2d ring1 = new Pose2d(8.3, 1.00, Rotation2d.fromDegrees(0));//heading 68 deg
    Pose2d shoot1 = new Pose2d(2.4, 3.25, Rotation2d.fromDegrees(-45));//heading 57 deg
    Pose2d ring2Intermediary = new Pose2d(5.3, 2.1, Rotation2d.fromDegrees(7));//heading 57 deg
    Pose2d ring2 = new Pose2d(8.3, 2.43, Rotation2d.fromDegrees(30));//heading 85 deg
    Pose2d shoot2 = new Pose2d(2.4, 3.25, Rotation2d.fromDegrees(-45));
    Pose2d ring3Intermediary = new Pose2d(2, 3.6, Rotation2d.fromDegrees(-10));
    Pose2d ring3 = new Pose2d(2.67, 4.12, Rotation2d.fromDegrees(-25));

    @Override
    public void runAuto() {
        SmartDashboard.putString("BlueOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = false;
            swerveBrake = true; 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > 0.85 && timer.get() < 1.15){
                runShooter = true;
            } else {
                runShooter = false;
            }

            if (timer.get() >= 1.15){
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose, Rotation2d.fromDegrees(-70)),
                        Waypoint.fromHolonomicPose(ring1,Rotation2d.fromDegrees(0))));
                runShooter = false;
                timer.reset();  
                step = Step.ring1;   
                runIntake = true;
            }               
        }
        else if(step == Step.ring1){
            driving = true;
            swerveBrake = false;
            runShooter = false;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                trajectoryConfig = new TrajectoryConfig(6, 3);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(ring1),
                            Waypoint.fromHolonomicPose(shoot1)));
                runShooter = false;
                timer.reset();    
                step = Step.shot1;
            }
        }
        else if(step == Step.shot1){
            driving = true;
            runShooter = false;
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();
                step = Step.shootBeforeRing2;
            }  
        }
        else if (step == Step.shootBeforeRing2){
            if(timer.get() < 0.2){
                driving = false;
                runShooter = true;
                armCommand = ArmCommanded.shotMap;
            }

            else {
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(-40)),
                        Waypoint.fromHolonomicPose(ring2Intermediary, Rotation2d.fromDegrees(0)),
                        Waypoint.fromHolonomicPose(ring2,Rotation2d.fromDegrees(0))));
            timer.reset();    
            runShooter = false;
            step = Step.ring2;
            }

        }
        else if (step == Step.ring2){
            driving = true;
            runShooter = false;
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                trajectoryConfig = new TrajectoryConfig(5, 2.5);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(230)),
                            Waypoint.fromHolonomicPose(shoot2,Rotation2d.fromDegrees(150))));
                timer.reset();    
                step = Step.shot2;
            }
        }
        else if (step == Step.shot2){
            driving = true;
            runShooter = false;
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();    
                step = Step.shootBeforeRing3;
            }
        }
        else if (step == Step.shootBeforeRing3){

            if(timer.get() < 0.2){
                driving = false;
                runShooter = true;
                armCommand = ArmCommanded.shotMap;
            }

            else {
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(ring3Intermediary,Rotation2d.fromDegrees(0)),
                        Waypoint.fromHolonomicPose(ring3,Rotation2d.fromDegrees(0))));
            timer.reset();    
            runShooter = false;
            step = Step.end;
            }


        }
        else if (step == Step.ring3){
            driving = true;
            armCommand = ArmCommanded.shotMap;
            runShooter = true;
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();    
                step = Step.end;
            }

        }
        else if (step == Step.end){
            driving = false;
            armCommand = ArmCommanded.shotMap;
            runShooter = true;
        }
        else {
            runShooter = false;
            driving = false;
            runIntake = false;
            swerveBrake = true;
        }

        SmartDashboard.putString("Step_step", step.toString());
        if (driving) {
            swerveBrake = false;
            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            swerveBrake = true;
        }

        visualizePath();
    }

    @Override
    public void reset() {
        super.reset();
        swerveBrake = false;
        step = Step.start;
        seedPose = false;
    }
}