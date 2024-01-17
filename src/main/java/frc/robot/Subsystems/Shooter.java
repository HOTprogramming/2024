package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import frc.robot.RobotState;
import com.ctre.phoenix6.hardware.TalonFX;



public class Shooter implements SubsystemBase {
    TalonFX flywheel;
    RobotState robotState;

    public Shooter(RobotState robotState) {
        this.robotState = robotState;
        flywheel = new TalonFX(FLYWHEEL_CAN);
    }

    @Override
    public void updateState() {
        if (robotState.getShooterOn()) {
            flywheel.set(FLYWHEEL_MAX_SPEED);
        }
    }

    @Override
    public void reset() {
        flywheel.setPosition(0);
    }
}
