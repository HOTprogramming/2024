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

    public Pose2d ring1 = new Pose2d(8.8, 7.05, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot1 = new Pose2d(10.74, 7.3, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(12, 7.1, Rotation2d.fromDegrees(175));
    public Pose2d almostring2 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring2 = new Pose2d(9.75, 5.89, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring2 = new Pose2d(9, 5.4, Rotation2d.fromDegrees(180));

    public Pose2d ring2 = new Pose2d(8.8, 5.4, Rotation2d.fromDegrees(225));
    public Pose2d almostshoot2 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(11.6, 6, Rotation2d.fromDegrees(175));
    public Pose2d almostring3 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring3 = new Pose2d(9.8, 4.8, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring3 = new Pose2d(10, 3.75, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(8.8, 3.74, Rotation2d.fromDegrees(235));
    public Pose2d almostshoot3 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot3 = new Pose2d(12, 6, Rotation2d.fromDegrees(-175));

    public NewAuto(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.15, 6.5, Rotation2d.fromDegrees(150)); //15.15

        seedPose = true;



        
    }

    @Override
    public void runAuto() {

        if(step == Step.start){

            driving = true;
            trajectoryConfig = new TrajectoryConfig(5, 4);
            trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(2));
            trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(new Pose2d(13.85, 7.3, Rotation2d.fromDegrees(180))),
                        Waypoint.fromHolonomicPose(new Pose2d(10, 7.05, Rotation2d.fromDegrees(180))),
                        Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(180)),
                        Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(150))));
            timer.reset();
            step = Step.ring1;

            
        } else if (step == Step.ring1) {
            if (timer.get() > (trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds() / 2)) {
                armCommand = ArmCommanded.shotMap;
            }
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;

                timer.reset();
                step = Step.shoot1;
                
                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(5));

                trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(robotState.getDrivePose()),
                        Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-75)),
                        Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(-20))));
            }
        } else if (step == Step.shoot1) {
            if (timer.get() > 1.2){
                runShooter = false;
                driving = true;
                armCommand = ArmCommanded.zero;
                timer.reset();
                step=Step.ring2;
            }
            runShooter = true;
        
        } else if(step == Step.ring2){


            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.ring3;

                trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(8));

                trajectoryGenerator.generate(trajectoryConfig, 
                List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(160)),
                        Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(-80)),
                        Waypoint.fromHolonomicPose(new Pose2d(11.5, 4.5, Rotation2d.fromDegrees(200))),
                        Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(90))));


            // Not under stage

            // trajectoryGenerator.generate(trajectoryConfig, 
            //     List.of(Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(160)),
            //             Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(-90)),
            //             Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(-20))));

            }  
        } else {
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                driving = false;
            }
        }

        if (robotState.getDrivePose().getX() < 11) {
            runIntake = true;
        } else {
            runIntake = false;
        }


        if (driving) {
            swerveBrake = false;

            SmartDashboard.putBoolean("Step_Driving", true);

            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            SmartDashboard.putBoolean("Step_Driving", false);

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