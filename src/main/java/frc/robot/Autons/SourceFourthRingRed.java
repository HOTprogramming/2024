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
public class SourceFourthRingRed extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        beforeShot1,
        shot1,
        ring3,
        beforeShot2,
        shot2,
        ring2,
        beforeShot3,
        shot3,
        ring1,
        end;
    }

    public Step step = Step.start;   
    private double speed = 6;
    private double accel = 3; 

    public SourceFourthRingRed(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.2, 1.460, Rotation2d.fromDegrees(180));

        seedPose = true;
    }

    Pose2d ring2 = new Pose2d(8.0, 2.30, Rotation2d.fromDegrees(-130));
    Pose2d stage = new Pose2d(11.2, 4.15, Rotation2d.fromDegrees(180));
    Pose2d shoot = new Pose2d(13.54, 2.90, Rotation2d.fromDegrees(-138));
    Pose2d shootStage = new Pose2d(13.54, 2.90, Rotation2d.fromDegrees(-141.5));
    Pose2d ring3 = new Pose2d(8.2, 4.20, Rotation2d.fromDegrees(180));
    Pose2d out = new Pose2d(12.14, 1.5, Rotation2d.fromDegrees(180));
    Pose2d ring1 = new Pose2d(8.2, 0.9, Rotation2d.fromDegrees(180)); 

    @Override
    public void runAuto() {
        SmartDashboard.putString("RedOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = true;
            swerveBrake = false;

            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(1.5);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(shoot)));
                runShooter = false;
                unPackage = true;  
                armCommand = ArmCommanded.unPackage;
                timer.reset();  
                step = Step.beforeShot1;             

        }
        else if(step == Step.beforeShot1){
            driving = true;
            runShooter = false;

            if(timer.get() > 0.3){
                armCommand = ArmCommanded.shotMap; 
                robotState.setAutonHintXPos(calculateArmHint(shoot)); 
            }
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
                timer.reset();
                step = Step.shot1;
              
            }  
        }
        else if (step == Step.shot1){
            driving = false;
            armCommand = ArmCommanded.shotMap;
            robotState.setAutonHintXPos(-1);
            if(timer.get() > 0.05 && timer.get() < 0.2){
                runShooter = true;
            }
            else if(timer.get()<=0.05){

            }

            else {
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(ring2)));
                driving = true;
                armCommand = ArmCommanded.sourceAutoRed;
                runShooter = false;
                timer.reset();  
                runIntake = true;
                step = Step.ring2;   
            }
        }
        else if(step == Step.ring2){ 

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring2),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(shootStage)));
                armCommand = ArmCommanded.sourceAutoRed;       
                runShooter = false;
                timer.reset();  
                step = Step.beforeShot2;   
                runIntake = true;              
            }
        }
        else if (step == Step.beforeShot2){

            if(robotState.getDrivePose().getX() > 12.64){
                armCommand = ArmCommanded.shotMap;
                robotState.setAutonHintXPos(calculateArmHint(shootStage));
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                runShooter = false;
                timer.reset();  
                step = Step.shot2;   
                runIntake = true;  
                driving = false;
            }
        }
        else if (step == Step.shot2){
            driving = false;
            armCommand = ArmCommanded.shotMap;
            robotState.setAutonHintXPos(-1);
            if(timer.get() > 0.05 && timer.get() < 0.2){
                runShooter = true;
            }
            else if(timer.get()<=0.05){

            }
            else {
                trajectoryConfig = new TrajectoryConfig(speed, accel);
                trajectoryConfig.setEndVelocity(0);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(shootStage),
                            Waypoint.fromHolonomicPose(stage),
                            Waypoint.fromHolonomicPose(ring3)));
            timer.reset();
            driving = true;
            runShooter = false;
            armCommand = ArmCommanded.sourceAutoRed;
            step = Step.ring3;
            }
        }
        else if(step == Step.ring3){ 

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring3),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(shootStage)));
                armCommand = ArmCommanded.sourceAutoRed;       
                runShooter = false;
                timer.reset();  
                step = Step.beforeShot3;   
                runIntake = true;              
            }
        }
        else if (step == Step.beforeShot3){

            if(robotState.getDrivePose().getX() > 12.64){
                armCommand = ArmCommanded.shotMap;
                robotState.setAutonHintXPos(calculateArmHint(shootStage));
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                runShooter = false;
                timer.reset();  
                step = Step.shot3;   
                runIntake = true;  
                driving = false;
            }
        }
        else if (step == Step.shot3){
            driving = false;
            armCommand = ArmCommanded.shotMap;
            robotState.setAutonHintXPos(-1);
            if(timer.get() > 0.1 && timer.get() < 0.3){
                runShooter = true;
            }
            else if(timer.get()<=0.1){

            }
            else{
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(2);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shootStage),
                        Waypoint.fromHolonomicPose(out),
                        Waypoint.fromHolonomicPose(ring1)));
               
            timer.reset();    
            runIntake = true;   
            driving = true;     
            step = Step.ring1;
            }
        }
        else if (step == Step.ring1){
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();  
                step = Step.end;   
            }
        }
        else if (step == Step.end){
            driving = false;
            armCommand = ArmCommanded.sourceAutoRed;
            runIntake = true;
            runShooter = false;
            
        }
        else {
            runShooter = false;
            driving = false;
            runIntake = true;
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

