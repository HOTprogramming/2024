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
public class SourceRed extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        ring1,
        ring2,
        shot1,
        ring3,
        shot2,
        end;
    }

    public Step step = Step.start;   
    private double speed = 6;
    private double accel = 3;

    public SourceRed(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(15.2, 1.63, Rotation2d.fromDegrees(180));

        seedPose = true;
    }

    Pose2d lobToRing1 = new Pose2d(11.8, 0.77, Rotation2d.fromDegrees(-120));
    Pose2d ring1 = new Pose2d(8.3, 0.9, Rotation2d.fromDegrees(180));
    Pose2d lobRing1Ring2 = new Pose2d(9.54, 1.54, Rotation2d.fromDegrees(-160));
    Pose2d ring2 = new Pose2d(8.3, 2.56, Rotation2d.fromDegrees(170));
    Pose2d shoot = new Pose2d(12.34, 2.28, Rotation2d.fromDegrees(-140));
    Pose2d ring3 = new Pose2d(8.31, 4.03, Rotation2d.fromDegrees(90));

    @Override
    public void runAuto() {
        SmartDashboard.putString("RedOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = true;
            swerveBrake = false; 
            armCommand = ArmCommanded.shotMap;

            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose),
                        Waypoint.fromHolonomicPose(lobToRing1),
                        Waypoint.fromHolonomicPose(ring1,Rotation2d.fromDegrees(180))));
                runShooter = false;
                timer.reset();  
                step = Step.ring1;   
                runIntake = true;              

        }
        else if(step == Step.ring1){ 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring1, Rotation2d.fromDegrees(0)),
                        Waypoint.fromHolonomicPose(lobRing1Ring2),
                        Waypoint.fromHolonomicPose(ring2,Rotation2d.fromDegrees(180))));
                runShooter = false;
                timer.reset();  
                step = Step.ring2;   
                runIntake = true;     
            }         

        }
        else if(step == Step.ring2){ 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring2, Rotation2d.fromDegrees(-30)),
                        Waypoint.fromHolonomicPose(shoot, Rotation2d.fromDegrees(30))));
                runShooter = false;
                timer.reset();  
                step = Step.shot1;   
                runIntake = true;              
            }
        }
        else if(step == Step.shot1){ 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(shoot, Rotation2d.fromDegrees(-150)),
                        Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(120))));
                runShooter = false;
                timer.reset();  
                step = Step.ring3;   
                runIntake = true;              
            }
        }
        else if(step == Step.ring3){ 
            armCommand = ArmCommanded.shotMap;

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
            trajectoryConfig = new TrajectoryConfig(speed, accel);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(ring3, Rotation2d.fromDegrees(-50)),
                        Waypoint.fromHolonomicPose(shoot, Rotation2d.fromDegrees(30))));
                runShooter = false;
                timer.reset();  
                step = Step.shot2;   
                runIntake = true;              
            }
        }
        else if (step == Step.shot2){
            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                runShooter = false;
                timer.reset();  
                step = Step.end;   
                runIntake = true;  
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
