package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class RobotState {
    private Pose2d drivePose;
    private boolean atTargetPose;

    private Pose2d[] visionMeasurements;
    private double[] visionTimestamps;
    private Matrix<N3, N2> visionStdevs;


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

    public void setVisionMeasurements(Pose2d[] visionMeasurements) {
        this.visionMeasurements = visionMeasurements;
    }


    public Pose2d[] getVisionMeasurements() {
        return visionMeasurements;
    }

    public void setVisionTimestamps(double[] visionTimestamps) {
        this.visionTimestamps = visionTimestamps;
    }

    public double[] getVisionTimestamps() {
        return visionTimestamps;
    }

    public void setVisionStdevs(Matrix<N3, N2> visionStdevs) {
        this.visionStdevs = visionStdevs;
    }

    public Matrix<N3, N2> getVisionStdevs() {
        return visionStdevs;
    }
}
