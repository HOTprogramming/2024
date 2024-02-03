package frc.robot;

import frc.robot.Subsystems.Arm.armDesiredPos;

public interface RobotCommander {
    public abstract boolean getRunShooter();
    public abstract double getTargetDriveSpeed();
    public abstract double getTargetArmSpeed();
    public abstract armDesiredPos armPosition();
    public abstract boolean getShooterIntake();
}