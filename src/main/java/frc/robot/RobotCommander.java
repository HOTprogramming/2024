package frc.robot;

//import frc.robot.Subsystems.Arm.armDesiredPos;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.utils.trajectory.RotationSequence;
public interface RobotCommander {
    public enum DriveMode {
        percent,
        stateDrive
    };
    
    /**
     * Drivetrain command
     * @return Percents of max speed in every axis (Strafe, Foreward, Turn). Values are (-1 to 1)
     * 
     */
    public double[] getDrivePercentCommand();

    public State getDriveState();

    public RotationSequence.State getDriveRotationState();

    /**
     * Drivetrain command
     * @return Current driving mode
     */
    public DriveMode getDriveMode();

    public boolean getBrakeCommand();

    public Pose2d getRefrenceTolerances();

    public Pose2d getOdomretryOverride();
    public boolean getResetRobotPose();

    public boolean getPidgeonReset();

    public int getAngleSnapCommand();
    public abstract boolean getIntake();    
    public abstract boolean getRunFeeder();
    public boolean getLockPoseCommand();
    public abstract boolean getRunShooter();
    public abstract boolean increaseLeftTargetSpeed();
    public abstract boolean decreaseLeftTargetSpeed();
    public abstract boolean increaseRightTargetSpeed();
    public abstract boolean decreaseRightTargetSpeed();
    public abstract double getTargetDriveSpeed();
    public boolean getLockSpeakerCommand();
    //public abstract armDesiredPos armPosition();
    public abstract double getTargetArmSpeed();
    public boolean runArm();
    public boolean zeroArm();
    public boolean setShoot();

}