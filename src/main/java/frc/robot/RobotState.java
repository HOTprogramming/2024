package frc.robot;

import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ConstantsFolder.ConstantsBase;
//import frc.robot.Subsystems.Arm.armDesiredPos;
import frc.robot.Subsystems.Camera.CameraPositions;
import frc.robot.Subsystems.CameraMeasurment;

public class RobotState {
    private ConstantsBase constants;
    private Alliance alliance = Alliance.Red;

    private Pose2d drivePose;
    private boolean atTargetPose;

    private Map<CameraPositions, CameraMeasurment> visionMeasurements;
    private double poseToSpeaker;
    private Translation2d velocity;
    
    private boolean shooterOn;
    private boolean intakeOn;
    private boolean feederOn;
    private double armPos;
    private double extendPos;
    private double shooterPos;
 
    private boolean feederStop = false;
 //   private armDesiredPos stateArmPos;
    public RobotState(ConstantsBase constants) {
        this.constants = constants;
    }

    public ConstantsBase getConstants() {
        return this.constants;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
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

    public void setDriveVelocity(Translation2d velocity){
        this.velocity = velocity;
    }

    public Translation2d getDriveVelocity(){
        return velocity;
    }

    public void setPoseToSpeaker(double poseToSpeaker){
        this.poseToSpeaker = poseToSpeaker;
    }

    public double getPoseToSpeaker(){
        return poseToSpeaker;
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
        public void setIntakeOn(boolean intakeOn) {
        this.intakeOn = intakeOn;
    }
       public void setFeederOn(boolean feederOn) {
        this.feederOn = feederOn;
    }


    public void setVisionMeasurements(Map<CameraPositions, CameraMeasurment> visionMeasurements) {
        this.visionMeasurements = visionMeasurements;
    }


    public Map<CameraPositions, CameraMeasurment> getVisionMeasurements() {
        return visionMeasurements;
    }

    public void setArmPos(double armPos){
        this.armPos = armPos;
    }

    public double getArmPos(){
        return armPos;
    }

    public boolean getShooterOn() {
        return shooterOn;
    }

    public void setFeederStopped(boolean feederStop) {
        this.feederStop = feederStop;
    }   

    public boolean getFeederStopped() {
        return feederStop;
    }

    public void setExtendPos(double extendPos){
        this.extendPos = extendPos;
    }

    public double getExtendPos(){
        return extendPos;
    }

    public void setShooterPos(double shooterPos){
        this.shooterPos = shooterPos;
    }

    public double getShooterPos(){
        return shooterPos;
    }
}
