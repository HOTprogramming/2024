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
import org.photonvision.targeting.TargetCorner;

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
import edu.wpi.first.math.geometry.Transform2d;
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
import java.util.List;
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
    PhotonCamera frontCamera = new PhotonCamera("front_camera");
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
    private Map<CameraPositions, List<PhotonTrackedTarget>> targetsSeenByCamera  = new EnumMap<>(CameraPositions.class);
    CameraPositions allowMultiTag;

    private int minimumTagsSeenByAnyCamera;
    private int lastMinimumTagsSeenByAnyCamera = 0;

    private int loopsPast;

    // docs https://docs.photonvision.org/ 

    boolean frontDetects  = frontCamera.getLatestResult().hasTargets();;
    int frontPipeline;
    double noteX;
    double noteY;
    Translation2d noteVector;
    PhotonTrackedTarget frontThing;
    Transform2d noteTransform;
    double noteDistance;
    //double FRONTHeartBeat;
    //double previousFRONTHeartBeat;

    public Camera(RobotState robotState) {

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

        this.robotState = robotState;
    }

    private Optional<EstimatedRobotPose> updateCameraMeasurment(CameraPositions key, CameraConstant constant, PhotonCamera camera, DoubleArrayPublisher publisher, PhotonPoseEstimator estimator, double lastEstTimestamp) {
        
        var visionEst = estimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        targetsSeenByCamera.put(key,camera.getLatestResult().targets);

        if (newResult) {
            lastEstTimestamp = latestTimestamp;
        } else {
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
                if(!(key == CameraPositions.FRONT && frontPipeline == 1)) {
                    cameraMeasurements.put(key,updateCameraMeasurment(key, constant, cameras.get(key), publishers.get(key), photonPoseEstimators.get(key), lastEstTimestamps.get(key)));
                }
            }
        }); 

        targetsSeenByCamera.forEach((key, list) -> {
            if (list.size() > minimumTagsSeenByAnyCamera) {
                minimumTagsSeenByAnyCamera = list.size();
            }
            SmartDashboard.putNumber("Camera" + key + "sees:", list.size());
        });

        allowMultiTag = CameraPositions.BACK;

        if (targetsSeenByCamera.containsKey(CameraPositions.BACK) && targetsSeenByCamera.get(CameraPositions.BACK).size() >=2 ) {
            allowMultiTag = CameraPositions.BACK;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.RIGHT) && targetsSeenByCamera.get(CameraPositions.RIGHT).size() >=2 ) {
            allowMultiTag = CameraPositions.RIGHT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.LEFT) && targetsSeenByCamera.get(CameraPositions.LEFT).size() >=2 ) {
            allowMultiTag = CameraPositions.LEFT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.FRONT) && targetsSeenByCamera.get(CameraPositions.FRONT).size() >=2 ) {
            allowMultiTag = CameraPositions.FRONT;
        } 

        cameraMeasurements.forEach((key,measurement) -> {
            if (measurement.isPresent()) {
                cameraStdDeviations.put(key,getEstimationStdDevs(measurement.get().estimatedPose.toPose2d(), cameras.get(key), constants.cameraConstants.get(key), photonPoseEstimators.get(key),allowMultiTag == key));
            }
        });

        robotState.putTargetsSeenByCamera(targetsSeenByCamera);


        robotState.setVisionMeasurements(cameraMeasurements);
        robotState.setCameraStdDeviations(cameraStdDeviations);



        //Obj Detection starts here
        frontDetects = frontCamera.getLatestResult().hasTargets();
        SmartDashboard.putBoolean("CAMERA: Front Camera sees anything", frontDetects);

        frontPipeline = frontCamera.getPipelineIndex();
        SmartDashboard.putBoolean("CAMERA: Front Camera Object Detection?", frontPipeline == 1);
        SmartDashboard.putNumber("CAMERA: Pipeline # for front camera", frontPipeline);

        robotState.setNoteDetected(frontPipeline == 1 && frontDetects);

        frontThing = frontCamera.getLatestResult().getBestTarget();

        if(frontDetects && frontPipeline == 1 && frontThing!=null){
            
            noteX = -frontThing.getYaw();
            robotState.setNoteYaw(Rotation2d.fromDegrees(-noteX));
            noteY = frontThing.getPitch();
            SmartDashboard.putNumber("CAMERA: Note X angle", noteX);
            SmartDashboard.putNumber("CAMERA: Note Y angle", noteY);

            noteDistance = PhotonUtils.calculateDistanceToTargetMeters(constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getZ(), 0.0, constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getRotation().getY(), Math.toRadians(noteY));
            noteVector = new Translation2d(noteDistance, 0);
            // noteVector = new Translation2d(-(constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getZ())/Math.tan(Math.toRadians(noteY)), 0); 
            SmartDashboard.putNumber("CAMERA: Note distance to camera", noteDistance);
            robotState.setNoteDistance(noteDistance);

            noteVector = noteVector.rotateBy(new Rotation2d(Math.toRadians(noteX) + robotState.getDrivePose().getRotation().getRadians()));
            noteVector = noteVector.plus(new Translation2d(constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getX(), constants.cameraConstants.get(CameraPositions.FRONT).getTransform().getY()).rotateBy(new Rotation2d(robotState.getDrivePose().getRotation().getRadians())));
            noteTransform = new Transform2d(noteVector, new Rotation2d(Math.toRadians(noteX)));

            SmartDashboard.putNumber("CAMERA: Note distance to robot", noteVector.getDistance(new Translation2d(0,0)));

            SmartDashboard.putNumber("CAMERA: Note X to robot", noteVector.getX());
            SmartDashboard.putNumber("CAMERA: Note Y to robot", noteVector.getY());

            robotState.setNotePose(noteTransform);
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, CameraConstant constant, PhotonPoseEstimator estimator, boolean allowMultiTag) {
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
        if (allowMultiTag && numTags > 1) estStdDevs = constant.getMultiTagStdDevs();;
        // Increase std devs based on (average) distance
        if (numTags > 1 && avgDist > 6.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public void setFrontCameraPipeline(int pipeline){ //sets the pipeline of the front camera
        frontCamera.setPipelineIndex(pipeline);
    }

    @Override
    public void init(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void teleop(RobotCommander commander) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enabled'");
    }

    @Override
    public void cameraLights() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disabled'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }
}