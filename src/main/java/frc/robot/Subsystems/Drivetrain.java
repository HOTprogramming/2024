package frc.robot.Subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import com.ctre.phoenix6.hardware.TalonFX;


public class Drivetrain implements SubsystemBase {
    RobotState robotState;
    TalonFX driveMotor;
    

    public Drivetrain(RobotState robotState) {
        this.robotState = robotState;
        driveMotor = new TalonFX(DRIVE_CAN);
    }

    @Override
    public void updateState() {
        robotState.setDriveSpeed(driveMotor.getVelocity().getValueAsDouble());
    }
    
    @Override
    public void enabled(RobotCommander commander) {
        
    }

    @Override
    public void disabled() {
       driveMotor.stopMotor();
    }

    @Override
    public void reset() {
        driveMotor.stopMotor();
    }
}
