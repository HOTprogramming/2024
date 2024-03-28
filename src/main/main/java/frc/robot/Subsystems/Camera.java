package frc.robot.Subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon.ProtobufPhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import java.beans.VetoableChangeListener;
import java.io.IOException;
import java.time.Month;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.xml.crypto.Data;

import frc.robot.Robot;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;
import frc.robot.ConstantsFolder.ConstantsBase.CameraConstant;

public class Camera implements SubsystemBase {

    public enum CameraPositions {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }

    ConstantsBase.Camera constants;
    RobotState robotState;

    // front
    PhotonCamera frontCamera;
    PhotonPipelineResult result;
    PhotonTrackedTarget frontBestTarget;
    double frontSampleTime;
    Pose2d frontCamEstimation;
    Matrix<N3, N1> frontDevs = VecBuilder.fill(1, 1, 1);


    PhotonCameraSim simFrontCam;

    // left
    PhotonCamera leftCamera;
    PhotonPipelineResult leftResult;
    PhotonTrackedTarget leftBestTarget;
    double leftSampleTime;
    Pose2d leftCamEstimation;
    Matrix<N3, N1> leftDevs = VecBuilder.fill(1, 1, 1);


    PhotonCameraSim simLeftCam;

    // right
    PhotonCamera rightCamera;
    PhotonPipelineResult rightResult;
    PhotonTrackedTarget rightBestTarget;
    double rightSampleTime;
    Pose2d rightCamEstimation;
    Matrix<N3, N1> rightDevs = VecBuilder.fill(1, 1, 1);


    PhotonCameraSim simRightCam;

    // rear
    PhotonCamera rearCamera;
    PhotonPipelineResult rearResult;
    PhotonTrackedTarget rearBestTarget;
    double rearSampleTime;
    Pose2d rearCamEstimation;
    Matrix<N3, N1> rearDevs = VecBuilder.fill(1, 1, 1);


    PhotonCameraSim simRearCam;

    
    AprilTagFieldLayout tags;
    VisionSystemSim simVision;
    SimCameraProperties globalShutterProperties;

    boolean tempSimBool = false;
    boolean drawWireframes = false; // resource heavy

    Nat<N3> rows = new Nat<N3>() {

        @Override
        public int getNum() {
            return 3;
        }
        
    };
    Nat<N4> colls = new Nat<N4>() {

        @Override
        public int getNum() {
            return 4;
        }
        
    };

    Matrix<N3, N4> stDevs = new Matrix(rows, colls);
    double[] timestamps = new double[] {-1, -1, -1 ,-1};
    Pose2d[] poses = new Pose2d[] {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };



    
    // NetworkTables
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Vision");
    StringPublisher fieldTypePublisher = table.getStringTopic(".type").publish();

    DoubleArrayPublisher frontCameraPub;
    DoubleArrayPublisher leftCameraPub;
    DoubleArrayPublisher rightCameraPub;
    DoubleArrayPublisher rearCameraPub;


    Map<CameraPositions, Optional<EstimatedRobotPose>> cameraMeasurements = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, Matrix<N3, N1>> cameraStdDeviations = new EnumMap<>(CameraPositions.class);

    Map<CameraPositions, PhotonCamera> cameras = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, DoubleArrayPublisher> publishers = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, PhotonPoseEstimator> photonPoseEstimators = new EnumMap<>(CameraPositions.class); 
    Map<CameraPositions, Double> lastEstTimestamps = new EnumMap<>(CameraPositions.class); 

    private int minimumTagsSeenByAnyCamera;
    private int lastMinimumTagsSeenByAnyCamera = 0;

    private int loopsPast;

    // docs https://docs.photonvision.org/ 

    boolean frontDetects;
    int frontPipeline;
    double noteX;
    double noteY;
    Translation2d noteVector;
    PhotonTrackedTarget frontThing;

    public Camera(RobotState robotState) {
        this.robotState = robotState;

        constants = robotState.getConstants().getCameraConstants();
        
        try {
            tags = AprilTagFieldLayout.loadFromResource(("/org/Apriltags/2024-crescendo.json"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // simulation setup
        globalShutterProperties = new SimCameraProperties();
        globalShutterProperties.setFPS(60);
        globalShutterProperties.setCalibration(1920, 1080, Rotation2d.fromDegrees(70));
        simVision = new VisionSystemSim("SimVision");
        simVision.addAprilTags(tags);

        CameraConstant cameraConstant = constants.cameraConstants.get(CameraPositions.FRONT);
        if (cameraConstant != null) {
                cameras.put(CameraPositions.FRONT, new PhotonCamera(cameraConstant.getName()));
                publishers.put(CameraPositions.FRONT, table.getDoubleArrayTopic("Front_Camera").publish());
                PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.FRONT), cameraConstant.getTransform());
                estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                photonPoseEstimators.put(CameraPositions.FRONT, estimator);
                lastEstTimestamps.put(CameraPositions.FRONT, -1.0);
            if (tempSimBool) {
                simFrontCam = new PhotonCameraSim(frontCamera, globalShutterProperties);
                simVision.addCamera(simFrontCam, cameraConstant.getTransform());

                simFrontCam.enableRawStream(true); // localhost:1181
                simFrontCam.enableProcessedStream(true); // localhost:1182
                simFrontCam.enableDrawWireframe(drawWireframes);
            }
        }

        cameraConstant = constants.cameraConstants.get(CameraPositions.LEFT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.LEFT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.LEFT, table.getDoubleArrayTopic("Left_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.LEFT), cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.LEFT, estimator);
                lastEstTimestamps.put(CameraPositions.LEFT, -1.0);
            if (tempSimBool) {
                simLeftCam = new PhotonCameraSim(leftCamera, globalShutterProperties);
                simVision.addCamera(simLeftCam, cameraConstant.getTransform());

                simLeftCam.enableRawStream(true);
                simLeftCam.enableProcessedStream(true);
                simLeftCam.enableDrawWireframe(drawWireframes);
            }
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.RIGHT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.RIGHT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.RIGHT, table.getDoubleArrayTopic("Right_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.RIGHT), cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.RIGHT, estimator);
                lastEstTimestamps.put(CameraPositions.RIGHT, -1.0);
            if (tempSimBool) {
                simRightCam = new PhotonCameraSim(rightCamera, globalShutterProperties);
                simVision.addCamera(simRightCam, cameraConstant.getTransform());

                simRightCam.enableRawStream(true);
                simRightCam.enableProcessedStream(true);
                simRightCam.enableDrawWireframe(drawWireframes);
            }
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.BACK);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.BACK, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.BACK, table.getDoubleArrayTopic("Rear_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.BACK), cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                lastEstTimestamps.put(CameraPositions.BACK, -1.0);
            photonPoseEstimators.put(CameraPositions.BACK, estimator);
           if (tempSimBool) {
                simRearCam = new PhotonCameraSim(rearCamera, globalShutterProperties);
                simVision.addCamera(simRearCam, cameraConstant.getTransform());

                simRearCam.enableRawStream(true);
                simRearCam.enableProcessedStream(true);
                simRearCam.enableDrawWireframe(drawWireframes);
            }
        }
    }

    private Optional<EstimatedRobotPose> updateCameraMeasurment(CameraPositions key, CameraConstant constant, PhotonCamera camera, DoubleArrayPublisher publisher, PhotonPoseEstimator estimator, double lastEstTimestamp) {
                
        var visionEst = estimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        if (newResult) 
            lastEstTimestamp = latestTimestamp;
        else {
            publisher.set(null);
        }
            visionEst.ifPresent(est -> {
                publisher.set(new double[] {est.estimatedPose.toPose2d().getX(),
                est.estimatedPose.toPose2d().getY(),
                est.estimatedPose.toPose2d().getRotation().getDegrees()});
            });
            return visionEst;
     }

    @Override
    public void updateState() {

        minimumTagsSeenByAnyCamera = 0;

        if (tempSimBool) {
            if (robotState.getDrivePose() != null) {
                simVision.update(robotState.getDrivePose());
            }
        }

        constants.cameraConstants.forEach((key,constant) -> {
            if (constant != null) {
                cameraMeasurements.put(key,updateCameraMeasurment(key, constant, cameras.get(key), publishers.get(key), photonPoseEstimators.get(key), lastEstTimestamps.get(key)));
                if (cameraMeasurements.get(key).isPresent()) {
                    cameraStdDeviations.put(key,getEstimationStdDevs(cameraMeasurements.get(key).get().estimatedPose.toPose2d(), cameras.get(key), constant, photonPoseEstimators.get(key)));
                    List<PhotonTrackedTarget> tagsUsed = cameraMeasurements.get(key).get().targetsUsed;
                    if (tagsUsed.size() > minimumTagsSeenByAnyCamera) {
                        minimumTagsSeenByAnyCamera = tagsUsed.size();
                    }
                    SmartDashboard.putNumber("Seen by " + key.name(), tagsUsed.size());
                }
            }
        });

        if(lastMinimumTagsSeenByAnyCamera != minimumTagsSeenByAnyCamera && loopsPast > 50){
            SmartDashboard.putNumber("minimumTagsSeenByAnyCamera",minimumTagsSeenByAnyCamera);
            if (minimumTagsSeenByAnyCamera >=2 ) {
                robotState.setOneTag(false);
                robotState.setTwoTags(true);
                robotState.setNoTag(false);
            } else if (minimumTagsSeenByAnyCamera == 1) {
                robotState.setOneTag(true);
                robotState.setTwoTags(false);
                robotState.setNoTag(false);
            } else {
                robotState.setOneTag(false);
                robotState.setTwoTags(false);
                robotState.setNoTag(true);
            }
            loopsPast = 0;
        } else{
            loopsPast++;
        }

        lastMinimumTagsSeenByAnyCamera = minimumTagsSeenByAnyCamera;


        robotState.setVisionMeasurements(cameraMeasurements);
        robotState.setCameraStdDeviations(cameraStdDeviations);

        frontDetects = frontCamera.getLatestResult().hasTargets();
        SmartDashboard.putBoolean("CAMERA: Front Camera sees anything", frontDetects);

        frontPipeline = frontCamera.getPipelineIndex();
        SmartDashboard.putBoolean("CAMERA: Front Camera Object Detection?", frontPipeline == 2);
        SmartDashboard.putNumber("CAMERA: Pipeline # for front camera", frontPipeline);

        if(frontDetects && frontPipeline == 2){
            frontThing = frontCamera.getLatestResult().getBestTarget();
            noteX = -frontThing.getYaw();
            noteY = frontThing.getPitch();
            SmartDashboard.putNumber("CAMERA: Note X angle", noteX);
            SmartDashboard.putNumber("CAMERA: Note Y angle", noteY);

            noteVector = new Translation2d(-(constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getZ()/39.37)/Math.tan(noteY), 0);
            SmartDashboard.putNumber("CAMERA: Note distance", noteVector.getDistance(noteVector));

            noteVector.rotateBy(new Rotation2d(Math.toRadians(noteX) + robotState.getDrivePose().getRotation().getRadians()));
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, CameraConstant constant, PhotonPoseEstimator estimator) {
        var estStdDevs = constant.getSingleTagStdDevs();
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = constant.getSingleTagStdDevs();;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void enabled(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enabled'");
    }

    @Override
    public void disabled() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disabled'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    public Translation2d notePose() {
        return noteVector;
    }

    public boolean noteDetected(){
        return frontDetects && frontCamera.getPipelineIndex() == 2;
    }
}