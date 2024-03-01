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

public class Right4Note extends AutonBase {

    enum Step {
        preload,
        start,
        shoot0,
        shootFirst,
        ring1,
        driveshoot1, 
        shoot1,
        ring2,
        driveshoot2, 
        shoot2,
        ring3,
        driveshoot3, 
        shoot3,
        driveRingOne,
        end
    }

    public Step step = Step.start;

    public Pose2d ring1 = new Pose2d(8.15, 6.8, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(11.9, 6, Rotation2d.fromDegrees(175));

    public Pose2d ring2 = new Pose2d(8.15, 5.48, Rotation2d.fromDegrees(190));

    public Pose2d firstShot = new Pose2d(13.7, 6.7, Rotation2d.fromDegrees(155));

    public Pose2d underStageShot = new Pose2d(11.8,4.75,Rotation2d.fromDegrees(188));

    public Right4Note(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.3, 6.5, Rotation2d.fromDegrees(150)); //15.15

        seedPose = true;

        setupTraj();
    }

    public void setupTraj(){
        trajectoryConfig = new TrajectoryConfig(5, 3);
        trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(3));
        trajectoryGenerator.generate(trajectoryConfig, 
            List.of(Waypoint.fromHolonomicPose(startPose),
                    Waypoint.fromHolonomicPose(firstShot)));
    }

    @Override
    public void runAuto() {
        if(step == Step.start){
            armCommand = ArmCommanded.preload;
            driving = false;

            if(timer.get() > 1){
                runShooter = true;
            }


            if(timer.get() > 1.5){
                timer.reset();
                step = Step.preload;
                            
                setupTraj();

                runShooter = false;
            }
        
        } else if(step == Step.preload){
            driving = true;
            runIntake = true;

            armCommand = ArmCommanded.protect;

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = false;
                driving = false;
                runIntake = true;
                step = Step.shootFirst;
                timer.reset();

                timer.reset();
            }
        } else if(step == Step.shootFirst){
            driving = false;

            runShooter = true;

            if(timer.get() > .5){
                driving = true;

                timer.reset();
                step = Step.driveRingOne;

                trajectoryConfig = new TrajectoryConfig(5, 3);

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(firstShot),
                    Waypoint.fromHolonomicPose(ring1)
                    ));
                            
                runShooter = false;
            }
        } else if(step == Step.driveRingOne){
            armCommand = ArmCommanded.zero;

            runIntake = true;

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.driveshoot1;
                
                timer.reset();

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring1),
                    Waypoint.fromHolonomicPose(shoot1)
                ));
            }
        } else if(step == Step.driveshoot1){
            armCommand = ArmCommanded.auton;

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                driving = false;
                step = Step.shoot1;
                timer.reset();
            }
        } else if(step == Step.shoot1){
            runShooter = true;

            if(timer.get() > .5){
                runShooter = false;
                step = Step.ring2;

                timer.reset();

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(160)),
                    Waypoint.fromHolonomicPose(ring2)
                ));
            }
        } else if(step == Step.ring2){
            driving = true;
            armCommand = ArmCommanded.zero;
            runIntake = true;

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                step = Step.driveshoot2;

                timer.reset();

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(20)),
                    Waypoint.fromHolonomicPose(shoot1)
                ));
            }
        } else if(step == Step.driveshoot2){
            armCommand = ArmCommanded.auton;
            
            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                driving = false;
                step = Step.shoot2;
                timer.reset();
            }
        } else if(step == Step.shoot2){
            runShooter = true;
        } else {
            driving = false;
            runShooter = false;
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
        setupTraj();
    }
}