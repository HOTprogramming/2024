package frc.robot.Autons;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;

public abstract class AutonBase {
    Timer timer = new Timer();
    
    RobotState robotState;

    public boolean runShooter;
    public double driveSpeed;
    public double armPos;


    public AutonBase(RobotState robotState){
        this.robotState = robotState;
    }

    public abstract void runAuto();
    public abstract void reset();
}
