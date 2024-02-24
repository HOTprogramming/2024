package frc.robot.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Subsystems.Arm.ArmCommanded;

public class NewAuto extends AutonBase {

    enum Step {
        unpackage,
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

    public Pose2d ring1 = new Pose2d(8.44, 7.05, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot1 = new Pose2d(10.74, 7.3, Rotation2d.fromDegrees(180));
    public Pose2d shoot1 = new Pose2d(12, 7.1, Rotation2d.fromDegrees(170));
    public Pose2d almostring2 = new Pose2d(10.32, 6.7, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring2 = new Pose2d(9.75, 5.89, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring2 = new Pose2d(9, 5.4, Rotation2d.fromDegrees(180));

    public Pose2d ring2 = new Pose2d(8.44, 5.4, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot2 = new Pose2d(10.73, 6.8, Rotation2d.fromDegrees(180));
    public Pose2d shoot2 = new Pose2d(11.6, 7, Rotation2d.fromDegrees(165));
    public Pose2d almostring3 = new Pose2d(10.72, 6.5, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostring3 = new Pose2d(9.4, 4.8, Rotation2d.fromDegrees(180));
    public Pose2d almostalmostalmostring3 = new Pose2d(10, 3.75, Rotation2d.fromDegrees(180));
    public Pose2d ring3 = new Pose2d(8.44, 3.74, Rotation2d.fromDegrees(180));
    public Pose2d almostshoot3 = new Pose2d(9.73, 6.5, Rotation2d.fromDegrees(180));
    public Pose2d shoot3 = new Pose2d(12, 7, Rotation2d.fromDegrees(155));

    public NewAuto(RobotState robotState) {
        super(robotState);
        // startPose = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180));
        startPose = new Pose2d(15.15, 6.75, Rotation2d.fromDegrees(140)); //15.15

        
        // startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        seedPose = true;

    }

    @Override
    public void runAuto() {

        if (step == Step.unpackage) {
            if (timer.get() > .2) {
                step = Step.start;
            }
            runIntake = true;

        } else if(step == Step.start){
            armCommand = ArmCommanded.shotMap;
            driving = false;

            if(robotState.getShooterOn()){
                step = Step.shoot0;
                timer.reset();
            }
        } else if (step == Step.shoot0){
            runShooter = true;

            if(timer.get() > 1){
                timer.reset();
                step = Step.ring1;

                generateTrajectory(constants.AUTON_DEFAULT_MAX_VELOCITY_METERS, 1, .8, 0, List.of(startPose, betweenrings, almostring1, almostalmostring1, ring1));
            }
        } else if(step == Step.ring1){
            driving = true;
            runIntake = true;
            runShooter = false;

            if(checkTime()){
                step = Step.driveshoot1;
                generateTrajectory(List.of(robotState.getDrivePose(), shoot1));

                timer.reset();

            }
        } else if(step == Step.driveshoot1){

            if(checkTime()){
                driving = false;
                step = Step.shoot1;
                timer.reset();
            }
        } else if (step == Step.shoot1) {
            runShooter = true;
            if (timer.get() > 1.2) {
                runShooter = false;
                driving = true;
                step = Step.ring2;
                timer.reset();
                generateTrajectory(List.of(robotState.getDrivePose(), almostring2, almostalmostring2, almostalmostalmostring2, ring2));
            }

        } else if (step == Step.ring2) {
            if (checkTime()) {
                timer.reset();
                generateTrajectory(List.of(robotState.getDrivePose(), almostshoot2, shoot2));
                step = Step.driveshoot2;
            }
        } else if (step == Step.driveshoot2) {
            if (checkTime() || robotState.getAtTargetPose()) {
                driving = false;
                step = Step.shoot2;
                timer.reset();
            }
        } else if (step == Step.shoot2) {
            runShooter = true;
            if (timer.get() > 1.2) {
                runShooter = false;
                driving = true;
                step = Step.ring3;
                timer.reset();
                generateTrajectory(List.of(robotState.getDrivePose(), almostring3, almostalmostring3, almostalmostalmostring3, ring3));
            }

        } else if (step == Step.ring3) {
            if (checkTime()) {
                timer.reset();
                generateTrajectory(List.of(robotState.getDrivePose(), almostshoot3, shoot3));
                step = Step.driveshoot3;
            }
        } else if (step == Step.driveshoot3) {
            if (checkTime()) {
                driving = false;
                step = Step.shoot3;
                timer.reset();
            }
        } else if (step == Step.shoot3) {
            runShooter = true;
            if (timer.get() > 1.2) {
                runIntake = false;
                armCommand = ArmCommanded.none;
                runShooter = false;
                step = Step.end;
            }
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
        step = Step.unpackage;
        if (robotState.getVisionTimestamps()[3] != -1) {
            startPose = robotState.getVisionMeasurements()[3];
        }
        seedPose = false;
    }
}