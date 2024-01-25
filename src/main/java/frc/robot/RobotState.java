package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
    private Pose2d drivePose;
    private boolean atTargetPose;

    private Pose2d visionMeasurement;
    private double visionTimestamp;

     /**
     * Set new drive pose
     * 
     * @param drivePose New drive pose
     */
    public void setDrivePose(Pose2d drivePose) {
        this.drivePose = drivePose;
    }

    /**
     * Get drive pose
     * 
     * @return Current drive pose
     */
    public Pose2d getDrivePose() {
        return drivePose;
    }

    /**
     * Set target state (within tolerances)
     * 
     * @param atTargetPose At refrence 
     */
    public void setAtTargetPose(Boolean atTargetPose) {
        this.atTargetPose = atTargetPose;
    }

    /**
     * Get within target tolerances
     * 
     * @return within refrence pose
     */
    public boolean getAtTargetPose() {
        return atTargetPose;
    }

    public void setVisionMeasurement(Pose2d visionMeasurement) {
        this.visionMeasurement = visionMeasurement;
    }


    public Pose2d getVisionMeasurement() {
        return visionMeasurement;
    }

    public void setVisionTimestamp(double visionTimestamp) {
        this.visionTimestamp = visionTimestamp;
    }

    public double getVisionTimestamp() {
        return visionTimestamp;
    }
}
