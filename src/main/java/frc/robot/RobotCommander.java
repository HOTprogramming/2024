package frc.robot;

public interface RobotCommander {
    public abstract boolean getRunShooter();
    public abstract double getTargetDriveSpeed();
    public abstract double getTargetArmSpeed();
    public abstract double armPosition1();
    public abstract boolean getShooterIntake();
}