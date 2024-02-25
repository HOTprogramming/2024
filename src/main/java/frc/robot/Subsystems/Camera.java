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
import java.util.Map;
import java.util.Optional;

import javax.xml.crypto.Data;

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




    double currentTime;

    Map<CameraPositions, CameraMeasurment> cameraMeasurements = new EnumMap<>(CameraPositions.class);

    Map<CameraPositions, PhotonCamera> cameras = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, DoubleArrayPublisher> publishers = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, PhotonPoseEstimator> photonPoseEstimators = new EnumMap<>(CameraPositions.class); 

    // docs https://docs.photonvision.org/ 

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
                photonPoseEstimators.put(CameraPositions.FRONT, new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.FRONT), cameraConstant.getTransform()));
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
            photonPoseEstimators.put(CameraPositions.LEFT, new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.LEFT), cameraConstant.getTransform()));
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
            photonPoseEstimators.put(CameraPositions.RIGHT, new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.RIGHT), cameraConstant.getTransform()));

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
            photonPoseEstimators.put(CameraPositions.BACK, new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(CameraPositions.BACK), cameraConstant.getTransform()));
            if (tempSimBool) {
                simRearCam = new PhotonCameraSim(rearCamera, globalShutterProperties);
                simVision.addCamera(simRearCam, cameraConstant.getTransform());

                simRearCam.enableRawStream(true);
                simRearCam.enableProcessedStream(true);
                simRearCam.enableDrawWireframe(drawWireframes);
            }
        }
    }

    private CameraMeasurment updateCameraMeasurment(CameraPositions key, CameraConstant constant, PhotonCamera camera, DoubleArrayPublisher publisher, PhotonPoseEstimator estimator, double[]STDEV_GAIN) {
                
                CameraMeasurment measurement = null;
                result = camera.getLatestResult();
                
                if (robotState.getDrivePose() != null){
                    estimator.setReferencePose(robotState.getDrivePose());
                }
                Optional<EstimatedRobotPose> pose = estimator.update();
                if (result.hasTargets()) {
                    measurement = new CameraMeasurment(key.name());
                    measurement.setTimestamp(result.getTimestampSeconds());
                    measurement.setAmbiguity(result.getBestTarget().getPoseAmbiguity());
                    frontBestTarget = result.getBestTarget();
                    // if (pose.isPresent()) {
                    //     measurement.setPose(pose.get().estimatedPose.toPose2d());
                    // }
                    if (result.getMultiTagResult().estimatedPose.isPresent) {
                        Transform3d fieldToRobot = result.getMultiTagResult().estimatedPose.best.plus(constant.getTransform());
                        measurement.setPose(new Pose2d(fieldToRobot.getTranslation().getX(), fieldToRobot.getTranslation().getY(), new Rotation2d(fieldToRobot.getRotation().getZ())));
                        System.out.println("MultiTagFor: "+ key.name());
                    } else {
                        measurement.setPose(PhotonUtils.estimateFieldToRobotAprilTag(frontBestTarget.getBestCameraToTarget(), 
                                                                                    tags.getTagPose(frontBestTarget.getFiducialId()).get(), 
                                                                                    constant.getTransform()).toPose2d());
                        System.out.println("SingleTag: "+ key.name() + "ID: " + frontBestTarget.getFiducialId());
                    }



                    // for (int i = 0; i < 3; i++) {
                    //     measurement.setSdtDeviation(i, 0, frontBestTarget.getBestCameraToTarget().getTranslation().getNorm() * constants.STDEV_GAIN[i]);
                    // }

                    publisher.set(new double[] {
                        measurement.getPose().getX(),
                        measurement.getPose().getY(),
                        measurement.getPose().getRotation().getDegrees()
                    });
                } else {
                publisher.set(new double[] {
                        -10,
                        -10,
                        0
                    });
                }
                return measurement;
     }

    @Override
    public void updateState() {
        currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (tempSimBool) {
            if (robotState.getDrivePose() != null) {
                simVision.update(robotState.getDrivePose());
            }
        }

        constants.cameraConstants.forEach((key,constant) -> {
            if (constant != null) {
                CameraMeasurment measurment = updateCameraMeasurment(key, constant, cameras.get(key), publishers.get(key), photonPoseEstimators.get(key), timestamps);
                if (measurment != null) {
                    cameraMeasurements.put(key,measurment);
                } else {
                    cameraMeasurements.remove(key);
                }
            }
        });

        robotState.setVisionMeasurements(cameraMeasurements);
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
    
}
