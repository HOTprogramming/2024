package frc.robot.Subsystems;

import frc.robot.RobotCommander;

public interface SubsystemBase {
    public void updateState();
    public void init(RobotCommander commander);
    public void enabled(RobotCommander commander);
    public void disabled();
    public void reset();
}
