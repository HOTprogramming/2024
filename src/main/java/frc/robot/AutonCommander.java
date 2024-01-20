package frc.robot;

import frc.robot.Autons.AutonBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.trajectory.RotationSequence;

public class AutonCommander implements RobotCommander {
    RobotState robotState;
    AutonBase auto;
    

    public AutonCommander(RobotState robotState) {
        this.robotState = robotState;
        
    }

    public void setAuto(AutonBase selectedAuto) {
        this.auto = selectedAuto;
        this.auto.reset();
    }

    @Override
    public double[] getDrivePercentCommand() {
        return new double[] {0, 0 ,0};
    }

    @Override
    public State getDriveState() {
        return auto.holoDriveState;
    }


    @Override
    public RotationSequence.State getDriveRotationState() {
        return auto.rotationState;
    }

    @Override
    public DriveMode getDriveMode() {
        return DriveMode.stateDrive;
    }

    @Override
    public Pose2d getRefrenceTolerances() {
        return auto.refrenceTolerances;
    }

    @Override
    public Pose2d getOdomretryOverride() {
        return auto.startPose;
    }

    @Override
    public boolean getBrakeCommand() {
        return auto.swerveBrake;
    }
}
