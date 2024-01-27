package frc.robot;

import frc.robot.Autons.AutonBase;

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
    public boolean getRunShooter() {
        return auto.runShooter;
    }

    public boolean increaseLeftTargetSpeed() {
        return false;
    }

    public boolean decreaseLeftTargetSpeed() {
        return false;
    }

    public boolean increaseRightTargetSpeed() {
        return false;
    }

    public boolean decreaseRightTargetSpeed() {
        return false;
    }

    @Override
    public double getTargetDriveSpeed() {
        return auto.driveSpeed;
    }

    public double getRunArm() {
        return auto.armPos;
    }

}
