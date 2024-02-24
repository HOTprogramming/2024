package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.CompBotConstants.Intake;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.Waypoint;

public class NewAuto extends AutonBase {

    enum Step {
        preload,
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

    public Step step = Step.start;


    public Pose2d almostbetweenrings = new Pose2d(15.1, 7.5, Rotation2d.fromDegrees(170));
    public Pose2d betweenrings = new Pose2d(14, 7.6, Rotation2d.fromDegrees(170));
    public Pose2d almostring1 = new Pose2d(10.73, 7, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring1 = new Pose2d(9.44, 7.05, Rotation2d.fromDegrees(180));

    public Pose2d ring1 = new Pose2d(8.8, 7.31, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot1 = new Pose2d(10.74, 7.3, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(11.85, 6.25, Rotation2d.fromDegrees(171.5));
    public Pose2d almostring2 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring2 = new Pose2d(9.75, 5.89, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring2 = new Pose2d(9, 5.4, Rotation2d.fromDegrees(180));

    public Pose2d ring2 = new Pose2d(8.8, 5.4, Rotation2d.fromDegrees(225));
    public Pose2d almostshoot2 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(11.6, 6, Rotation2d.fromDegrees(175));
    public Pose2d almostring3 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring3 = new Pose2d(9.8, 4.8, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring3 = new Pose2d(10, 3.75, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(8.8, 3.74, Rotation2d.fromDegrees(-90));
    public Pose2d almostshoot3 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot3 = new Pose2d(12, 6, Rotation2d.fromDegrees(-175));

    public NewAuto(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.15, 6.5, Rotation2d.fromDegrees(150)); //15.15

        seedPose = true;



        
    }

    @Override
    public void runAuto() {
        if (step == Step.preload) {
            driving = false;
            armCommand = ArmCommanded.preload;
            if (robotState.getShooterOn()) {
                runShooter = true;
            }
            if (robotState.getShooterOn() && timer.get() > 1.7) {
                step = Step.start;
            }
        } else if(step == Step.start){
            armCommand = ArmCommanded.zero;
            driving = true;
            trajectoryConfig = new TrajectoryConfig(5, 3);
            trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(2));
            trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(new Pose2d(14.39, 7.55, Rotation2d.fromDegrees(180))),
                        Waypoint.fromHolonomicPose(new Pose2d(12.54, 7.55, Rotation2d.fromDegrees(180))),
                        Waypoint.fromHolonomicPose(new Pose2d(11, 7.31, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(ring1)));
            timer.reset();
            step = Step.ring1;

            
        } else if (step == Step.ring1) {
            if (robotState.getDrivePose().getX() < 11.5) {
                armCommand = ArmCommanded.zero;
                runShooter = false;
                runIntake = true;
            } else {
                runIntake = false;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0)),
                    Waypoint.fromHolonomicPose(shoot1)
                    ));
                armCommand = ArmCommanded.auton;
                timer.reset();
                step = Step.driveshoot1;
                
                
            }
        } else  if (step == Step.driveshoot1) {
            if (robotState.getDrivePose().getX() < 11.5) {
                runIntake = true;
            } else {
                runIntake = false;
            }

            if(robotState.getAtTargetPose()){
                runIntake = false;
                driving = false;
                timer.reset();
                step = Step.shoot1;
                
                
            }
            
        } else if (step == Step.shoot1) {

            runShooter = true;
       
            if (timer.get() > 1.2){
                runShooter = false;
                driving = true;
                armCommand = ArmCommanded.zero;

                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(8));
                trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(shoot1),
                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-75)),
                    Waypoint.fromHolonomicPose(new Pose2d(9.11, 4.42, Rotation2d.fromDegrees(166))),
                    Waypoint.fromHolonomicPose(new Pose2d(11.3, 3.97, Rotation2d.fromDegrees(170))),
                    Waypoint.fromHolonomicPose(new Pose2d(12.1, 5.18, Rotation2d.fromDegrees(180))),
                    Waypoint.fromHolonomicPose(shoot1)));
                timer.reset();
                step=Step.ring2;
            }
        } else if(step == Step.ring2){

            if (robotState.getDrivePose().getX() < 11.5) {
                runIntake = true;
            } else {
                runIntake = false;
            }

            if (robotState.getDrivePose().getX() > 11.5) {
                armCommand = ArmCommanded.auton;
            } else {
                armCommand = ArmCommanded.zero;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.shoot2;
                armCommand = ArmCommanded.auton;

            } 
        } else if (step == Step.shoot2) {
            runShooter = true;
            if (timer.get() > 1.2){
                runShooter = false;
                armCommand = ArmCommanded.zero;
                step = Step.ring3;

                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(8));

                trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(160)),
                        Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(-90)),
                        Waypoint.fromHolonomicPose(new Pose2d(11.7, 4.2, Rotation2d.fromDegrees(-140))),
                        Waypoint.fromHolonomicPose(shoot1)));

                        
                // Not under stage

                // trajectoryGenerator.generate(trajectoryConfig, 
                //     List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(160)),
                //             Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(-90)),
                //             Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(-20))));


            }
        } else if (step == Step.ring3) {
            if (robotState.getDrivePose().getX() < 11.5) {
                runIntake = true;
            } else {
                runIntake = false;
            }

            if (robotState.getDrivePose().getX() > 11.5) {
                armCommand = ArmCommanded.auton;
            } else {
                armCommand = ArmCommanded.zero;
            }
            
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                armCommand = ArmCommanded.auton;
                runIntake = false;
                driving = false;
                timer.reset();
                step = Step.shoot3;
            }
        } else if (step == Step.shoot3) {
            runShooter = true;
            if (timer.get() > 1.2){
                runShooter = false;
                armCommand = ArmCommanded.zero;
                step = Step.end;
            }

        } else {
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
            }
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
        step = Step.preload;
        seedPose = false;
    }
}