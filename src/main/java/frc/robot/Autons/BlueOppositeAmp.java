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

        startPose = new Pose2d(1.45, 4.63, Rotation2d.fromDegrees(-70));

        seedPose = true;
    }
    
    Pose2d ring1 = new Pose2d(8.34, 0.93, Rotation2d.fromDegrees(0));
    Pose2d ring1Intermediary = new Pose2d(4.7, 1.8, Rotation2d.fromDegrees(-35));
    Pose2d shoot1 = new Pose2d(3.49, 3.12, Rotation2d.fromDegrees(-33));
    Pose2d ring2Intermediary = new Pose2d(5.3, 1.9, Rotation2d.fromDegrees(0));
    Pose2d ring2 = new Pose2d(8.28, 2.80, Rotation2d.fromDegrees(0));
    Pose2d shoot2 = new Pose2d(3.49, 3.12, Rotation2d.fromDegrees(-33));

    @Override
    public void runAuto() {
        SmartDashboard.putString("BlueOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = false;
            swerveBrake = true; 
            armCommand = ArmCommanded.preload;
            unPackage = true;

            if(timer.get() > 0.95 && timer.get() < 1.15){
                runShooter = true;
            } else {
                runShooter = false;
            }
            
            if (timer.get() >= 1.15){
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(ring1Intermediary),
                        Waypoint.fromHolonomicPose(ring1)));
                runShooter = false;
                timer.reset();  
                armCommand = ArmCommanded.shotMap;
                step = Step.ring1;   
                runIntake = true;
                unPackage = false;
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
            robotState.setAutonHintXPos(calculateArmHint(shoot1));
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();
                step = Step.shootBeforeRing2;
            }  
        }
        else if (step == Step.shootBeforeRing2){
            if(timer.get() > 0.1 && timer.get() < 0.3){
                driving = false;
                runShooter = true;
                armCommand = ArmCommanded.shotMap;
            }
            else if(timer.get() <= 0.1){

            }
            else {
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot1),
                        Waypoint.fromHolonomicPose(ring2Intermediary),
                        Waypoint.fromHolonomicPose(ring2)));
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
                    List.of(Waypoint.fromHolonomicPose(ring2),
                            Waypoint.fromHolonomicPose(ring2Intermediary),
                            Waypoint.fromHolonomicPose(shoot2)));
                timer.reset();    
                step = Step.shot2;
            }
        }
        else if (step == Step.shot2){
            driving = true;
            runShooter = false;
            robotState.setAutonHintXPos(calculateArmHint(shoot2));
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();    
                step = Step.shootBeforeRing3;
            }
        }
        else if (step == Step.shootBeforeRing3){

            if(timer.get() > 0.1 && timer.get() < 0.3){
                driving = false;
                runShooter = true;
                armCommand = ArmCommanded.shotMap;
            }
            else if(timer.get() <= 0.1){

            }

            else {
            timer.reset();    
            runShooter = true;
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