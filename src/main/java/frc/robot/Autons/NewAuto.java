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

    public Pose2d ring1 = new Pose2d(8.4, 7.1, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot1 = new Pose2d(10.74, 7.3, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(12, 6.33, Rotation2d.fromDegrees(171.5));
    public Pose2d almostring2 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring2 = new Pose2d(9.75, 5.89, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring2 = new Pose2d(9, 5.4, Rotation2d.fromDegrees(180));

    public Pose2d ring2 = new Pose2d(8.65, 5.5, Rotation2d.fromDegrees(190));
    public Pose2d almostshoot2 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(11.6, 6, Rotation2d.fromDegrees(175));
    public Pose2d almostring3 = new Pose2d(10.72, 6, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring3 = new Pose2d(9.8, 4.8, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring3 = new Pose2d(10, 3.75, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(8.8, 3.74, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot3 = new Pose2d(10.73, 6.2, Rotation2d.fromDegrees(180));
    public Pose2d shoot3 = new Pose2d(12, 6, Rotation2d.fromDegrees(-175));

    public Pose2d underStageShot = new Pose2d(11.8,4.75,Rotation2d.fromDegrees(188));

    public NewAuto(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.3, 6.7, Rotation2d.fromDegrees(155)); //15.15

        seedPose = true;

        setupTraj();
    }

    public void setupTraj(){
        trajectoryConfig = new TrajectoryConfig(5, 3);
        trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(3));
        trajectoryGenerator.generate(trajectoryConfig, 
            List.of(Waypoint.fromHolonomicPose(startPose),
                    Waypoint.fromHolonomicPose(new Pose2d(14.39, 7.55, Rotation2d.fromDegrees(180))),
                    Waypoint.fromHolonomicPose(new Pose2d(12.54, 7.55, Rotation2d.fromDegrees(180))),
                    Waypoint.fromHolonomicPose(new Pose2d(11, 7.31, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
                    Waypoint.fromHolonomicPose(ring1)));
    }

    @Override
    public void runAuto() {
        if(step == Step.start){
            armCommand = ArmCommanded.preload;
            driving = false;

            if(timer.get() > 1)
            runShooter = true;


            if(timer.get() > 1.5){
                timer.reset();
                step = Step.preload;
                            
                setupTraj();

                runShooter = false;
            }
        
        } else if(step == Step.preload){
            driving = true;
            if(timer.get() > 1.5){
                runIntake = true;

            }
            armCommand = ArmCommanded.preload;

            if (timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()) {
                runShooter = false;
                driving = true;
                runIntake = true;
                step = Step.ring1;
                timer.reset();
                trajectoryConfig = new TrajectoryConfig(5, 3);

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0)),
                    Waypoint.fromHolonomicPose(shoot1)
                    ));
                armCommand = ArmCommanded.auton;
                timer.reset();
            }
        } else if (step == Step.ring1) {

            if(timer.get() > 2){
                runIntake = false;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.shoot1;

                driving = false;
            }
        } else if(step == Step.shoot1){
            runShooter = true;

            if(timer.get() > .5){
                driving = true;
                runShooter = false;
                runIntake = true;

                timer.reset();
                step = Step.ring2;

                armCommand = ArmCommanded.zero;

                trajectoryConfig = new TrajectoryConfig(5, 3);

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(shoot1, Rotation2d.fromDegrees(180)),
                    Waypoint.fromHolonomicPose(ring2)
                    ));
            }
        }else if(step == Step.ring2){
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.driveshoot2;

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring2),
                    Waypoint.fromHolonomicPose(new Pose2d(10.75,4,Rotation2d.fromDegrees(183))),
                    Waypoint.fromHolonomicPose(underStageShot)
                ));
            }
        } else if(step == Step.driveshoot2){
            armCommand = ArmCommanded.auton;

            if(timer.get() > 1.5){
                runIntake = false;
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.shoot2;

                driving = false;

                timer.reset();
            }
        } else if(step == Step.shoot2){
            runShooter = true;

            if(timer.get() > .5){
                timer.reset();
                step = Step.ring3;

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(underStageShot),
                    Waypoint.fromHolonomicPose(ring3)
                ));            

                driving = true;
            }
        }else if(step == Step.ring3){
            armCommand = ArmCommanded.zero;
            runIntake = true;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();
                step = Step.driveshoot3;

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(ring3),
                    Waypoint.fromHolonomicPose(underStageShot)
                ));            

                driving = true;
            }
        }else if(step == Step.driveshoot3){
            armCommand = ArmCommanded.auton;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.end;

                driving = false;

                timer.reset();
            }
        } else if(step == Step.shoot3){
            runShooter = true;

            if(timer.get() > .5){
                timer.reset();
                step = Step.end;

                trajectoryGenerator.generate(trajectoryConfig, List.of(
                    Waypoint.fromHolonomicPose(underStageShot),
                    Waypoint.fromHolonomicPose(ring3)
                ));            

                driving = true;
            }
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