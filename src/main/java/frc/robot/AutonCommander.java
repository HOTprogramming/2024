package frc.robot;

import frc.robot.Autons.AutonBase;
import frc.robot.Subsystems.Arm.armDesiredPos;

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

    @Override
    public double getTargetArmSpeed() {
        return auto.armSpeed;
    }
    
    @Override
    public armDesiredPos armPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'armPosition1'");
    }
    @Override
    public boolean getShooterIntake(){
        return false;
    }

}
