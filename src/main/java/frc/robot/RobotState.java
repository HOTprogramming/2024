package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ConstantsFolder.ConstantsBase;
//import frc.robot.Subsystems.Arm.armDesiredPos;

public class RobotState {
    private ConstantsBase constants;
    private Pose2d drivePose;
    private boolean atTargetPose;

    private Pose2d[] visionMeasurements;
    private double[] visionTimestamps;
    private Matrix<N3, N2> visionStdevs;


    private boolean shooterOn;
    private double armPos;
 //   private armDesiredPos stateArmPos;
    public RobotState(ConstantsBase constants) {
        this.constants = constants;
    }

    public ConstantsBase getConstants() {
        return this.constants;
    }


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

    // public void encoderCounts(double position){
    //     this.position = position;
    // }

    // public double getEncoderCounts(){
    //     return position;
    // }

    /**
     * Get within target tolerances
     * 
     * @return within refrence pose
     */
    public boolean getAtTargetPose() {
        return atTargetPose;
    }

    public void setShooterOn(boolean shooterOn) {
        this.shooterOn = shooterOn;
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

    public void setArmPos(double armPos){
        this.armPos = armPos;
    }

    public double getArmPos(){
        return armPos;
    }
}
