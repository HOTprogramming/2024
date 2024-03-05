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


public class Center4Note extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        drivetofirst,
        shoot1,
        driveto2,
        shoot2,
        driveto3,
        shoot3,
        end
    }

    public Step step = Step.start;   

    public Center4Note(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.27, 5.6, Rotation2d.fromDegrees(180)); //15.15

        seedPose = true;
    }

    Pose2d ring1 = new Pose2d(13.75, 5.6, Rotation2d.fromDegrees(180));
    Pose2d ring2 = new Pose2d(13.75, 4.12, Rotation2d.fromDegrees(200));
    Pose2d ring3 = new Pose2d(13.84, 6.84, Rotation2d.fromDegrees(150));

    @Override
    public void runAuto() {
        
        if(step == Step.start){
            armCommand = ArmCommanded.shotMap;
            driving = false;

            if(timer.get() > 1){
                runShooter = true;
            }

            if(timer.get() > 1.5){
                runIntake = true;

                driving = true;

                timer.reset();
                step = Step.drivetofirst;
                            
                trajectoryConfig = new TrajectoryConfig(3, 2);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(startPose),
                            Waypoint.fromHolonomicPose(ring1)));
                
                runShooter = false;
            }
        } else if(step == Step.drivetofirst){
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.shoot1;
                driving = false;
                timer.reset();
            }   
        } else if(step == Step.shoot1){
            runShooter = true;

            if(timer.get() > 1){
                driving = true;

                timer.reset();
                step = Step.driveto2;
                            
                trajectoryConfig = new TrajectoryConfig(3, 2);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(-45)),
                            Waypoint.fromHolonomicPose(new Pose2d(14.5, 5, Rotation2d.fromDegrees(200))),
                            Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(135))));
                
                runShooter = false;
            }
        } else if(step == Step.driveto2){
            armCommand = ArmCommanded.protect;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.shoot2;
                driving = false;
                timer.reset();            
            }
        }else if(step == Step.shoot2){
            runShooter = true;

            if(timer.get() > 1){
                driving = true;

                timer.reset();
                step = Step.driveto3;
                            
                trajectoryConfig = new TrajectoryConfig(3, 2);
                trajectoryGenerator.generate(trajectoryConfig,
                    List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(45)),
                            Waypoint.fromHolonomicPose(new Pose2d(14, 6, Rotation2d.fromDegrees(150))),
                            Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(135))));
                
                runShooter = false;
            }
        } else if(step == Step.driveto3){
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                step = Step.shoot3;
                driving = false;
                timer.reset();                  
            }
        } else if(step == Step.shoot3){
            runShooter = true;

            if(timer.get() > 1){
                driving = false;

                timer.reset();
                step = Step.end;
                
                runShooter = false;
            }
        } else {
            runShooter = false;
            driving = false;
            runIntake = false;
        }

        // runShooter = false;
        // runIntake = false;
        // armCommand = ArmCommanded.none;

        SmartDashboard.putString("Step_step", step.toString());
        if (driving) {
            swerveBrake = false;
            holoDriveState = trajectoryGenerator.getDriveTrajectory().sample(timer.get());
            rotationState = trajectoryGenerator.getHolonomicRotationSequence().sample(timer.get());
        } else {
            swerveBrake = true;
        }

        // driving = false;

        // if(step == Step.start){
        //     runShooter = true;

        //     if(timer.get() > 1){
        //         step = Step.driveto2;
        //         runShooter = false;
        //     }
        // } else {
        //     runIntake = true;
        // }


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