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

    @Override
    public double getTargetDriveSpeed() {
        return auto.driveSpeed;
    }

}
