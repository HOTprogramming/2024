package frc.robot.Subsystems;

import static frc.robot.Constants.Drivetrain.*;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;


public class Drivetrain extends SwerveDrivetrain implements SubsystemBase {
    RobotState robotState;
    

    public Drivetrain(RobotState robotState) {
        super(DRIVETRAIN_CONSTANTS, 
        FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS, BACK_RIGHT_MODULE_CONSTANTS);
        this.robotState = robotState;
    }

    @Override
    public void updateState() {
    }
    
    @Override
    public void enabled(RobotCommander commander) {
    }

    @Override
    public void disabled() {
    }

    @Override
    public void reset() {
    }
}
