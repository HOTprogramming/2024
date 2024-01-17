package frc.robot;

public class RobotState {
    private boolean currentShooterOn;


    /**
     * The current shooter state
     * 
     * @param bool New shooter state
     */
    public void setShooterOn(boolean bool) {
        currentShooterOn = bool;
    }

    /**
     * The current shooter state
     * 
     * @return Whether the shooter speed is above minimums
     */
    public boolean getShooterOn() {
        return currentShooterOn;
    }
}
