package frc.robot;

public class RobotState {
    private boolean shooterOn;
    private double driveSpeed;

    /**
     * Set the current shooter state
     * 
     * @param state New shooter state
     */
    public void setShooterOn(boolean shooterOn) {
        this.shooterOn = shooterOn;
    }

    /**
     * Get the current shooter state
     * 
     * @return Whether the shooter speed is above minimums
     */
    public boolean getShooterOn() {
        return shooterOn;
    }

    /**
     * Set new drive speed
     * 
     * @param driveSpeed New drive speed
     */
    public void setDriveSpeed(double driveSpeed) {
        this.driveSpeed = driveSpeed;
    }

    /**
     * Get drive speed
     * 
     * @return Current drive speed
     */
    public double getDriveSpeed() {
        return driveSpeed;
    }
}
