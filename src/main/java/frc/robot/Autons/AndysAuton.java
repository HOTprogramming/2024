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
public class AndysAuton extends AutonBase {
    // create steps based on desired function
    enum Step {
        start,
        two,
        three,
        four;
    }

    public Step step = Step.start;   

    public AndysAuton(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(1.14, 6.016, Rotation2d.fromDegrees(0)); //15.15

        seedPose = true;
    }
    
    Pose2d end = new Pose2d(7.8, 7.3, Rotation2d.fromDegrees(10));

    @Override
    public void runAuto() {
        SmartDashboard.putString("Andysenum", step.toString());
        
        if(step == Step.start){
            runShooter = false;
            driving = false;
            runIntake = false;
            swerveBrake = true; 

            
            trajectoryConfig = new TrajectoryConfig(6, 4);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(startPose, Rotation2d.fromDegrees(0)),
                        Waypoint.fromHolonomicPose(end,Rotation2d.fromDegrees(0))));
                driving = false;
                timer.reset();  
                step = Step.two;              
            
                            
        }
        else if (step == Step.two){
            robotState.setAutonHintXPos(3.76); //3.9 posetospeaker
            armCommand = ArmCommanded.shotMap; 
            driving = true;
            runIntake = false;
            swerveBrake = false;
            
            if(timer.get()<1){
                runShooter = false;
            }
            else if (timer.get()>=1){
                runShooter = true; 
            }

            if(timer.get() > trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds()){
                timer.reset();    
                step = Step.three;
            }

        } 
        else if(step == Step.three){
            runShooter = false;
            driving = false;
            runIntake = false;
            swerveBrake = true;    
        }
        else {
            runShooter = false;
            driving = false;
            runIntake = false;
            swerveBrake = true;
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