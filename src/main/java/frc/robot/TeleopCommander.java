package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopCommander implements RobotCommander {
    public static XboxController controller;

    RobotState robotState;
    double armPose;


    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;

        controller = new XboxController(0);
    }

    @Override
    public boolean getRunShooter() {
        return controller.getRightBumper();
    }

    public boolean increaseLeftTargetSpeed() {
        return controller.getAButtonPressed();
    }

    public boolean decreaseLeftTargetSpeed() {
        return controller.getBButtonPressed();
    }

    public boolean increaseRightTargetSpeed() {
        return controller.getXButtonPressed();
    }

    public boolean decreaseRightTargetSpeed() {
        return controller.getYButtonPressed();
    }

    @Override
    public double getTargetDriveSpeed() {
        return 0;
    }

    public double getRunArm() {
        if (controller.getLeftY() < 0.05 && controller.getLeftY() > -0.05) {
            return 0;
        } else {
            return controller.getLeftY();
        }
    }
}
