package frc.robot;

public class RobotState {
    private boolean shooterOn;
    private double drivePose;
    private double position;

    /**
     * Set the current shooter state
     * 
     * @param shooterOn New shooter state
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
     * Set new drive pose
     * 
     * @param drivePose New drive pose
     */
    public void setDrivePose(double drivePose) {
        this.drivePose = drivePose;
    }

    public void encoderCounts(double position){
        this.position = position;
    }

    public double getEncoderCounts(){
        return position;
    }

    /**
     * Get drive pose
     * 
     * @return Current drive pose
     */
    public double getDrivePose() {
        return drivePose;
    }
}
