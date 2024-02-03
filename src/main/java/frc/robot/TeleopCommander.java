package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Arm.armDesiredPos;

public class TeleopCommander implements RobotCommander {
    private static XboxController joysticks;

    RobotState robotState;


    armDesiredPos armSetXPos = armDesiredPos.zero;


    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;

        joysticks = new XboxController(0);
    }

    @Override
    public boolean getRunShooter() {
        return joysticks.getAButton();
    }

    @Override
    public double getTargetDriveSpeed() {
        return joysticks.getLeftY() * 0.05;
    }

    @Override
    public double getTargetArmSpeed() {
        return joysticks.getRightY() * 5;
    }

    @Override
    public armDesiredPos armPosition() {
        if(joysticks.getXButton()){
            armSetXPos = armDesiredPos.zero;
            return armSetXPos;
        } else if(joysticks.getBButton()){
            armSetXPos = armDesiredPos.shoot;
            return armSetXPos;
        }
        else{
        return armSetXPos;
        }
    }

    @Override
    public boolean getShooterIntake(){
        return joysticks.getRightBumper();
    }
}
