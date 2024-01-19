package frc.robot.Subsystems;

import static frc.robot.Constants.Shooter.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter implements SubsystemBase {
    RobotState robotState;
    TalonFX flywheel;
    

    public Shooter(RobotState robotState) {
        this.robotState = robotState;
        flywheel = new TalonFX(FLYWHEEL_CAN);
    }

    @Override
    public void updateState() {
        if (flywheel.getVelocity().getValueAsDouble() >= (FLYWHEEL_MAX_SPEED - FLYWHEEL_MAX_VELOCITY_ERROR)) {
            robotState.setShooterOn(true);
        } else {
            robotState.setShooterOn(false);
        }
    }
    
    @Override
    public void enabled(RobotCommander commander) {
        if (commander.getRunShooter()) {
            flywheel.set(FLYWHEEL_MAX_SPEED);
        } else {
            flywheel.set(0);
        }
    }

    @Override
    public void disabled() {
        flywheel.stopMotor();
    }

    @Override
    public void reset() {
        flywheel.stopMotor();
    }
}
