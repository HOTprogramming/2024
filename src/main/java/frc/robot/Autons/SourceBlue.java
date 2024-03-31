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
public class SourceBlue extends AutonBase {
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
        end;
    }

    public Step step = Step.start;   
    private double speed = 2.5;
    private double accel = 1.7;

    public SourceBlue(RobotState robotState) {
        super(robotState);

        startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        seedPose = true;
    }

    Pose2d lobToRing1 = new Pose2d(5.83, 1.319, Rotation2d.fromDegrees(0));
    Pose2d ring1 = new Pose2d(8.3, 0.9, Rotation2d.fromDegrees(0));
    Pose2d lobRing1Ring2 = new Pose2d(6.0, 1.54, Rotation2d.fromDegrees(0));
    Pose2d ring2 = new Pose2d(8.8, 2.56, Rotation2d.fromDegrees(30));
    Pose2d stage = new Pose2d(5.7, 4.3, Rotation2d.fromDegrees(0));
    Pose2d shoot = new Pose2d(2.8, 3.5, Rotation2d.fromDegrees(-40));
    Pose2d ring3 = new Pose2d(8.5, 4.20, Rotation2d.fromDegrees(0));

    @Override
    public void runAuto() {
        SmartDashboard.putString("RedOppositeAmpEnum", step.toString());
        
        if(step == Step.start){
            driving = true;
            swerveBrake = false;

            trajectoryConfig = new TrajectoryConfig(3, 2);
            trajectoryGenerator.generate(trajectoryConfig,
                List.of(Waypoint.fromHolonomicPose(new Pose2d(0,0,Rotation2d.fromDegrees(0))),
                Waypoint.fromHolonomicPose(new Pose2d(robotState.getNotePose().getX(), robotState.getNotePose().getY(), Rotation2d.fromDegrees(0)))
));
                runShooter = false;
                timer.reset();  
                step = Step.ring1;   
                runIntake = false;   
                armCommand = ArmCommanded.spitOut;           
        } else if(step == Step.ring1){

        } else {
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
