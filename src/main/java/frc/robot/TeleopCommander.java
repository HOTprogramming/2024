package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopCommander implements RobotCommander {
    private static XboxController joysticks;

    RobotState robotState;

    double armPos = 0;


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
    public double armPosition1() {
        if(joysticks.getXButton()){
            armPos = 0;
        } else if(joysticks.getBButton()){
            armPos = 2;
        }

        return armPos;

    }

    @Override
    public boolean getShooterIntake(){
        return joysticks.getRightBumper();
    }
}
