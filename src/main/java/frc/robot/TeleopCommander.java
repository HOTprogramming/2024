package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopCommander extends RobotCommander {
    private static XboxController joysticks;

    RobotState robotState;


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
        return joysticks.getLeftY();
    }
}
