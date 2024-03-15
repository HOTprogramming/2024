package frc.robot;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ConstantsFolder.ConstantsBase;
//import frc.robot.Subsystems.Arm.armDesiredPos;
import frc.robot.Subsystems.Camera.CameraPositions;

public class RobotState {
    private ConstantsBase constants;
    private Alliance alliance = Alliance.Blue;

    private Pose2d drivePose;
    private boolean atTargetPose;

    private Map<CameraPositions, Optional<EstimatedRobotPose>> visionMeasurements;
    
    private Map<CameraPositions, Matrix<N3, N1>> cameraStdDeviations;

    private double poseToSpeaker;
    private Translation2d velocity;
    
    private boolean shooterOn;
    private boolean intakeOn;
    private boolean feederOn;
    private double armPos;
    private double extendPos;
    private double shooterPos;
    private boolean beambreak;
    private boolean shooterAmpTrap;
    private boolean feederAmpTrap;
    private boolean twoTags;
    private boolean oneTag;
    private boolean noTag;
    private double autonHintXPos;
    private boolean oneFirst;
 
    private boolean feederStop = false;
    private Map<CameraPositions, List<PhotonTrackedTarget>> targetsSeenByCamera;
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

    public Map<CameraPositions, Matrix<N3, N1>> getCameraStdDeviations() {
        return cameraStdDeviations;
    }

    public void setCameraStdDeviations(Map<CameraPositions, Matrix<N3, N1>> cameraStdDeviations) {
        this.cameraStdDeviations = cameraStdDeviations;
    }

    public void setVisionMeasurements(Map<CameraPositions, Optional<EstimatedRobotPose>> visionMeasurements) {
        this.visionMeasurements = visionMeasurements;
    }


    public Map<CameraPositions, Optional<EstimatedRobotPose>> getVisionMeasurements() {
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

    public void setBeamBreak(boolean beambreak){
        this.beambreak = beambreak;
    }

    public boolean getBeamBreak(){
        return beambreak;
    }

    public void setShooterOnAmpTrap(boolean shooterAmpTrap){
        this.shooterAmpTrap = shooterAmpTrap;
    }

    public boolean getShooterOnAmpTrap(){
        return shooterAmpTrap;
    }

    public void setFeederOnAmpTrap(boolean feederAmpTrap){
        this.feederAmpTrap = feederAmpTrap;
    }

    public boolean getFeederOnAmpTrap(){
        return feederAmpTrap;
    }

    public void setAutonHintXPos(double autonHintXPos){
        this.autonHintXPos = autonHintXPos;
    }

    public double getAutonHintXPos(){
        return autonHintXPos;
    }

    public void setTwoTags(boolean twoTags){
        this.twoTags = twoTags;
    }
    public void setOneTag(boolean oneTag){
        this.oneTag = oneTag;
    }
    public void setNoTag(boolean noTag){
        this.noTag = noTag;
    }

    public boolean getTwoTags(){
        return twoTags;
    }
    public boolean getOneTag(){
        return oneTag;
    }
    public boolean getNoTag(){
        return noTag;
    }

    public void putTargetsSeenByCamera(Map<CameraPositions, List<PhotonTrackedTarget>> targetsSeenByCamera) {
        // TODO Auto-generated method stub
        this.targetsSeenByCamera = targetsSeenByCamera;
    }

    
    public Map<CameraPositions, List<PhotonTrackedTarget>> getTargetsSeenByCamera() {
        return targetsSeenByCamera;
    }

    public void setOneNoteFirst(boolean oneFirst){
        this.oneFirst = oneFirst;
    }

    public boolean getOneNoteFirst(){
        return oneFirst;
    }

}
