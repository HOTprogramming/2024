package frc.robot.Subsystems;

import frc.robot.RobotCommander;

public interface SubsystemBase {
    /**
     * Called in robot periodic. Should contain sensor and robotState update code
     */
    public void updateState();

    /**
     * Called in any init you want (must update robot.java)
     * @param commander the current commander, should not matter at a subsystem level
     */
    public void init(RobotCommander commander);

    /**
     * Called in any periodic where the subsystem is enabled (must update robot.java)
     * @param commander the current commander, should not matter at a subsystem level
     */
    public void teleop(RobotCommander commander);

    /**
     * Called in disabled init or periodic. Should set motor controll to off or breakmode
     */
    public void cameraLights();

    public void reset();
}
