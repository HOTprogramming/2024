package frc.robot;

public interface RobotCommander {
    public abstract boolean getRunShooter();
    public abstract boolean increaseLeftTargetSpeed();
    public abstract boolean decreaseLeftTargetSpeed();
    public abstract boolean increaseRightTargetSpeed();
    public abstract boolean decreaseRightTargetSpeed();
    public abstract double getTargetDriveSpeed();
    public abstract double getRunArm();
}