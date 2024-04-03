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
public class SourceCrazyBlue extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        ring1,
        ring2,
        beforeShot1,
        shot1,
        ring3,
        beforeShot2,
        shot2,
        firstSpit,
        beforeShot3,
        shot3,
        secondSpit,
        beforeShot4,
        shot4,
        end;
    }

    public Step step = Step.start;   
    private double speed = 7;
    private double accel = 4; 

    public SourceCrazyBlue(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(1.94, 1.319, Rotation2d.fromDegrees(0));

        seedPose = true;
    }

    Pose2d lobToRing1 = new Pose2d(5.83, 1.319, Rotation2d.fromDegrees(0));
    Pose2d ring1 = new Pose2d(8.1, 0.9, Rotation2d.fromDegrees(0));
    Pose2d lobRing1Ring2 = new Pose2d(5.6, 1.54, Rotation2d.fromDegrees(0));
    Pose2d ring2 = new Pose2d(9.0, 2.70, Rotation2d.fromDegrees(0));
    Pose2d stage = new Pose2d(5.7, 3.88, Rotation2d.fromDegrees(0));
    Pose2d shoot = new Pose2d(3.49, 3.12, Rotation2d.fromDegrees(-33));
    Pose2d shoot2 = new Pose2d(2.67, 2.93, Rotation2d.fromDegrees(-46.3));
    Pose2d ring3 = new Pose2d(8.2, 4.20, Rotation2d.fromDegrees(0));
    Pose2d firstSpit = new Pose2d(3.9, 1.00, Rotation2d.fromDegrees(-49.0));
    Pose2d secondSpit = new Pose2d(3.90, 1.58, Rotation2d.fromDegrees(-137.5));

    @Override
    public void runAuto() {
        SmartDashboard.putString("RedOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = true;
            swerveBrake = false;

            trajectoryConfig = new TrajectoryConfig(7, 4);
            trajectoryConfig.setEndVelocity(1.5);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(lobToRing1),
                        Waypoint.fromHolonomicPose(ring1,Rotation2d.fromDegrees(0))));
                runShooter = false;
                unPackage = true;  
                armCommand = ArmCommanded.unPackage;
                timer.reset();  
                step = Step.ring1;             

        }
        else if(step == Step.ring1){ 

            if(timer.get() > 0.8){
            armCommand = ArmCommanded.spitOut;
            runIntake = true;
            }

            if(robotState.getDrivePose().getX() > 4.2){
                armCommand = ArmCommanded.spitOut;
                runShooter = true;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(lobRing1Ring2),
                        Waypoint.fromHolonomicPose(ring2)));
                unPackage = false;
                runShooter = false;
                timer.reset();  
                step = Step.ring2;   
                runIntake = true;  
                armCommand = ArmCommanded.spitOut2; 

            }         

        }
        else if(step == Step.ring2){ 

            if(robotState.getDrivePose().getX() < 6.0){
                armCommand = ArmCommanded.spitOut2;
                runShooter = true;
              }
            else{
                runShooter = false;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring2),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(shoot)));
                runShooter = false;
                timer.reset();  
                runIntake = true;   
                armCommand = ArmCommanded.sourceAuto;
                step = Step.beforeShot1;   
            }
        }

        else if(step == Step.beforeShot1){
            driving = true;
            runShooter = false;

            if(robotState.getDrivePose().getX() < 4.5){
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
            if(timer.get() < 0.15){
                driving = false;
                runShooter = true;
                armCommand = ArmCommanded.shotMap;
                robotState.setAutonHintXPos(-1);
            }

            else {
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(0))));
                driving = true;
                armCommand = ArmCommanded.sourceAuto2;
                runShooter = false;
                timer.reset();  
                runIntake = true;
                step = Step.ring3;   
            }
        }
        else if(step == Step.ring3){ 

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(6, 3);
            trajectoryConfig.setEndVelocity(0);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring3),
                        Waypoint.fromHolonomicPose(stage),
                        Waypoint.fromHolonomicPose(shoot2)));
                armCommand = ArmCommanded.sourceAuto2;       
                runShooter = false;
                timer.reset();  
                step = Step.beforeShot2;   
                runIntake = true;  

            }
        }
        else if (step == Step.beforeShot2){

            if(robotState.getDrivePose().getX() < 4.5){
                armCommand = ArmCommanded.shotMap;
                robotState.setAutonHintXPos(calculateArmHint(shoot));
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
                trajectoryConfig = new TrajectoryConfig(6, 3);
                trajectoryConfig.setEndVelocity(0);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(shoot2),
                            Waypoint.fromHolonomicPose(firstSpit)));
            timer.reset();
            driving = true;
            runShooter = false;
            armCommand = ArmCommanded.shotMap;
            robotState.setAutonHintXPos(calculateArmHint(firstSpit));
            step = Step.firstSpit;
            }
        }

        else if (step == Step.firstSpit){

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                    timer.reset();
                    //step = Step.beforeShot3;
                    step = Step.end;            
                }

        }  

        else if (step == Step.end){
            if(timer.get() > 0.1){
            runShooter = true; 
            }
            robotState.setAutonHintXPos(-1);
            driving = false;
            armCommand = ArmCommanded.shotMap;
            runIntake = false;
            
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
