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
public class FourRedOppositeAmp extends AutonBase {
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

    public FourRedOppositeAmp(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.11, 3.3, Rotation2d.fromDegrees(-122)); //15.15

        seedPose = true;
    }
    
    Pose2d ring1 = new Pose2d(8.3, 0.9, Rotation2d.fromDegrees(180));//
    Pose2d ring1Intermediary = new Pose2d(10.9, 0.92, Rotation2d.fromDegrees(180));//
    Pose2d shoot1 = new Pose2d(13.58, 3.25, Rotation2d.fromDegrees(-138));//heading 57 deg
    Pose2d ring2Intermediary = new Pose2d(11.0, 1.8, Rotation2d.fromDegrees(173));//heading 57 deg
    Pose2d ring2 = new Pose2d(8.3, 2.56, Rotation2d.fromDegrees(170));//heading 85 deg
    Pose2d shoot2 = new Pose2d(14.8, 4.3, Rotation2d.fromDegrees(-138));

    Pose2d ring3Intermediary = new Pose2d(14.5, 3.9, Rotation2d.fromDegrees(-170));
    Pose2d ring3 = new Pose2d(13.96, 4.23, Rotation2d.fromDegrees(-150));

    @Override
    public void runAuto() {
        SmartDashboard.putString("RedOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = false;
            swerveBrake = true; 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > 0.75 && timer.get() < 1.1){
                runShooter = true;
            } else {
                runShooter = false;
            }

            if (timer.get() >= 1.1){
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose, Rotation2d.fromDegrees(-110)),
                        Waypoint.fromHolonomicPose(ring1Intermediary),
                        Waypoint.fromHolonomicPose(ring1,Rotation2d.fromDegrees(180))));
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

            if(timer.get() > 1){
                armCommand = ArmCommanded.shotMap;
            }

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
                List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(-140)),//subtract this from 180
                        Waypoint.fromHolonomicPose(ring2Intermediary, Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(ring2,Rotation2d.fromDegrees(180))));
            timer.reset();    
            runShooter = false;
            step = Step.ring2;
            }

        }
        else if (step == Step.ring2){
            driving = true;
            runShooter = false;
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                trajectoryConfig = new TrajectoryConfig(6, 3);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-40)),
                            Waypoint.fromHolonomicPose(shoot2,Rotation2d.fromDegrees(40))));
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
                List.of(Waypoint.fromHolonomicPose(shoot2, Rotation2d.fromDegrees(160)),
                        Waypoint.fromHolonomicPose(ring3)));
            timer.reset();    
            runShooter = false;
            step = Step.ring3;
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