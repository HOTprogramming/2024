package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import frc.robot.Subsystems.Arm.armDesiredPos;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AutonBase;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Arm.ArmCommanded;
import frc.robot.utils.trajectory.RotationSequence;

public class TeleopCommander implements RobotCommander {
    private static XboxController driver;
    private static XboxController operator;
    

    RobotState robotState;
    double armPose;
    // Camera camera = new Camera(robotState);

    double deadbands = 0.2;
    double LX;
    double LY;
    double RX;


  //  armDesiredPos armSetXPos = armDesiredPos.zero;


    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;

        driver = new XboxController(0);
        operator = new XboxController(1);

    }

    public void driverRumble() {
        if (robotState.getDriverRumble()) {
            driver.setRumble(RumbleType.kBothRumble, 0.7);

        } else {
            driver.setRumble(RumbleType.kBothRumble, 0.0);

        }
    }

    @Override
    public AutonBase getAuto(){
        return null;
    }

   @Override
   
    public double[] getDrivePercentCommand() {
        double leftY;
        double leftx;
        double rightx;

        if (robotState.getAlliance() == Alliance.Red) {
            if (Math.abs(driver.getLeftY()) > deadbands) {
                leftY = driver.getLeftY();
            } else {
                leftY = 0;
            }

            if (Math.abs(driver.getLeftX()) > deadbands) {
                leftx = driver.getLeftX();
            } else {
                leftx = 0;
            }

            if (Math.abs(driver.getRightX()) > deadbands) {
                rightx = -driver.getRightX();
            } else {
                rightx = 0;
            }            
        } else {
            if (Math.abs(driver.getLeftY()) > deadbands) {
                leftY = -driver.getLeftY();
            } else {
                leftY = 0;
            }

            if (Math.abs(driver.getLeftX()) > deadbands) {
                leftx = -driver.getLeftX();
            } else {
                leftx = 0;
            }

            if (Math.abs(driver.getRightX()) > deadbands) {
                rightx = -driver.getRightX();
            } else {
                rightx = 0;
            }
        }

        return new double[] {leftY, leftx, rightx};
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
        return operator.getLeftTriggerAxis() > .1;
    }
    public boolean getRunFeeder() {
        return (operator.getLeftTriggerAxis() > 0.01);
    }

    public boolean increaseLeftTargetSpeed() {
        // return operator.getYButtonPressed();
        return false;
    }

    public boolean decreaseLeftTargetSpeed() {
        // return operator.getXButtonPressed();
        return false;
    }

    public boolean increaseRightTargetSpeed() {
        // return operator.getBButtonPressed();
        return false;
    }

    public boolean decreaseRightTargetSpeed() {
        // return operator.getAButtonPressed();
        return false;
    }

    @Override
    public double getTargetArmSpeed() {
        return operator.getRightY() * 5;
    }

    // @Override
    // public armDesiredPos armPosition() {
    //     if(operator.getRightBumper()){
    //         armSetXPos = armDesiredPos.shoot;
    //         return armSetXPos;
    //     }
    //     else{
    //     armSetXPos = armDesiredPos.zero;
    //     return armSetXPos;
    //     }

    // }

    public ArmCommanded armCommanded(){
        if(operator.getLeftTriggerAxis() >= .1 && operator.getAButton() != true && operator.getBButton() != true && operator.getXButton() != true && operator.getYButton() != true){
            return ArmCommanded.shotMap;
        }
        else if(operator.getLeftBumper() && operator.getRightBumper()){
            if(operator.getBButton()){
                return ArmCommanded.trap;
            } else {
                return ArmCommanded.handoff;
            }
        }
        else if (operator.getAButton()){
            return ArmCommanded.close;
        }
        else if (operator.getYButton()){
            return ArmCommanded.protect;
        }
        else if (operator.getBButton() && !(operator.getRightTriggerAxis() >= .1)){
            return ArmCommanded.hailMary;
        }
        else if (operator.getRightStickButton()){
            return ArmCommanded.trapZero;
        }
        else if (operator.getLeftBumper()){
            return ArmCommanded.zero;
        }
        else if (operator.getXButton()) {
            return ArmCommanded.amp;
        }
        else{
            return ArmCommanded.none;
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

    @Override
    public boolean getIntake() {
        return operator.getRightTriggerAxis() > .1;
    }

    @Override
    public boolean setShoot() {
        return driver.getRightTriggerAxis() >= .1;
    }
    
     @Override
    public boolean getFeeder() {
        return operator.getRightTriggerAxis() > .1;
    }

    @Override
    public boolean climberUp() {
        if(  operator.getPOV() >= 315 || (operator.getPOV() <= 45 && 0 <= operator.getPOV()) ){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public boolean climberDown() {
        if(135 <= operator.getPOV() && operator.getPOV() <= 225){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public double trapArmFineControl() {
        if (driver.getLeftBumper())  {
            return 1;
        } else if (driver.getRightBumper())  {
            return -1;
        } else {
            return 0;
        }
    }

    @Override
    public boolean climberOverride() {
     if(operator.getRightBumper()){
        return true;
     }
     else{
        return false;
     }
    }

    @Override
    public boolean intakeOut() {
       if(operator.getBButton() && operator.getRightTriggerAxis() >= .1){
        return true;
     }
     else{
        return false;
     }
    }

    @Override
    public boolean getLockParallel() {
        return false;    
    }

    @Override
    public boolean getLockAmpCommand() {
        return driver.getXButton();
    }

    @Override
    public boolean extentionOveride() {
        return driver.getBButton();
    }

    @Override
    public boolean extentionZero() {
        return driver.getBButtonReleased();
    }

    public boolean getLockNote() {
        return driver.getYButton();
    }
}