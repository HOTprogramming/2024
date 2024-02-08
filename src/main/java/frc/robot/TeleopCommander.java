package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Arm.armDesiredPos;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.trajectory.RotationSequence;

public class TeleopCommander implements RobotCommander {
    private static XboxController driver;
    private static XboxController operator;


    RobotState robotState;
    double armPose;


    armDesiredPos armSetXPos = armDesiredPos.zero;


    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;

        driver = new XboxController(0);
        operator = new XboxController(1);

    }

   @Override
   
    public double[] getDrivePercentCommand() {
        return new double[] {-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()};
    }

    @Override
    public State getDriveState() {
        return null;
    }


    @Override
    public RotationSequence.State getDriveRotationState() {
        return null;
    }

    @Override
    public DriveMode getDriveMode() {
        if (driver.getLeftTriggerAxis() > 0) {
            return DriveMode.stateDrive;
        } else {
             return DriveMode.percent;
        }
       
    }

    @Override
    public boolean getRunShooter() {
        return driver.getRightTriggerAxis() > .25;
    }

    public boolean increaseLeftTargetSpeed() {
        return operator.getYButtonPressed();
    }

    public boolean decreaseLeftTargetSpeed() {
        return operator.getXButtonPressed();
    }

    public boolean increaseRightTargetSpeed() {
        return operator.getBButtonPressed();
    }

    public boolean decreaseRightTargetSpeed() {
        return operator.getAButtonPressed();
    }

    @Override
    public double getTargetDriveSpeed() {
        return operator.getLeftY();
    }

    @Override
    public double getTargetArmSpeed() {
        return operator.getRightY() * 5;
    }

    @Override
    public armDesiredPos armPosition() {
        if(operator.getRightBumper()){
            armSetXPos = armDesiredPos.shoot;
            return armSetXPos;
        }
        else{
        armSetXPos = armDesiredPos.zero;
        return armSetXPos;
        }

    }


    @Override
    public Pose2d getRefrenceTolerances() {
        return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }


    @Override
    public Pose2d getOdomretryOverride() {
        // will be camera stuff
        return null;
    }


    @Override
    public boolean getBrakeCommand() {
        return driver.getAButton();
    }

    @Override
    public int getAngleSnapCommand() {
        int theta = driver.getPOV();
        if (theta == 90 || theta == 270) {
            theta += 180;
        }
        SmartDashboard.putNumber("POV", theta);
        return theta;
    }

    @Override
    public boolean getPidgeonReset() {
        return driver.getStartButton();              
    }


    @Override
    public boolean getLockSpeakerCommand() {
        return (driver.getLeftTriggerAxis() > .1);
    }

    @Override
    public boolean getLockPoseCommand() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getResetRobotPose() {
        return driver.getBackButton();
    }


}
