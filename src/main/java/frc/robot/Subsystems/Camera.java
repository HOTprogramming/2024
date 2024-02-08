package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import java.io.IOException;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;

public class Camera implements SubsystemBase {
    ConstantsBase.Camera constants;
    RobotState robotState;

    // front
    PhotonCamera frontCamera;
    PhotonPipelineResult frontResult;
    PhotonTrackedTarget frontBestTarget;
    double frontSampleTime;
    Pose2d frontCamEstimation;

    PhotonCameraSim simFrontCam;

    // left
    PhotonCamera leftCamera;
    PhotonPipelineResult leftResult;
    PhotonTrackedTarget leftBestTarget;
    double leftSampleTime;
    Pose2d leftCamEstimation;

    PhotonCameraSim simLeftCam;

    // right
    PhotonCamera rightCamera;
    PhotonPipelineResult rightResult;
    PhotonTrackedTarget rightBestTarget;
    double rightSampleTime;
    Pose2d rightCamEstimation;

    PhotonCameraSim simRightCam;

    // rear
    PhotonCamera rearCamera;
    PhotonPipelineResult rearResult;
    PhotonTrackedTarget rearBestTarget;
    double rearSampleTime;
    Pose2d rearCamEstimation;

    PhotonCameraSim simRearCam;

    
    AprilTagFieldLayout tags;
    VisionSystemSim simVision;
    SimCameraProperties globalShutterProperties;

    boolean tempSimBool = true;
    boolean drawWireframes = false; // resource heavy

    // Nat<N3> rows = new Nat<N3>() {

    //     @Override
    //     public int getNum() {
    //         return 3;
    //     }
        
    // };
    // Nat<N2> colls = new Nat<N2>() {

    //     @Override
    //     public int getNum() {
    //         return 2;
    //     }
        
    // };


    // Matrix<N3, N2> stdevs = new Matrix(rows, colls);


    
    // NetworkTables
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Vision");
    StringPublisher fieldTypePublisher = table.getStringTopic(".type").publish();

    DoubleArrayPublisher frontCameraPub;
    DoubleArrayPublisher leftCameraPub;
    DoubleArrayPublisher rightCameraPub;
    DoubleArrayPublisher rearCameraPub;




    double currentTime;
    

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


        if (constants.HAS_FRONT_CAMERA) {
            frontCamera = new PhotonCamera(constants.FRONT_CAMERA_NAME);
            frontCameraPub = table.getDoubleArrayTopic("Front_Camera").publish();
        }
        if (constants.HAS_LEFT_CAMERA) {
            leftCamera = new PhotonCamera(constants.LEFT_CAMERA_NAME);
            leftCameraPub = table.getDoubleArrayTopic("Left_Camera").publish();
        }
        if (constants.HAS_RIGHT_CAMERA) {
            rightCamera = new PhotonCamera(constants.RIGHT_CAMERA_NAME);
            rightCameraPub = table.getDoubleArrayTopic("Right_Camera").publish();
        }
        if (constants.HAS_REAR_CAMERA) {
            rearCamera = new PhotonCamera(constants.REAR_CAMERA_NAME);
            rearCameraPub = table.getDoubleArrayTopic("Rear_Camera").publish();
        }

        // simulation setup
        if (tempSimBool) {
            System.out.println("test");
            globalShutterProperties = new SimCameraProperties();
            globalShutterProperties.setFPS(60);
            globalShutterProperties.setCalibration(1920, 1080, Rotation2d.fromDegrees(70));
            simVision = new VisionSystemSim("SimVision");
            simVision.addAprilTags(tags);


            if (constants.HAS_FRONT_CAMERA) {
                simFrontCam = new PhotonCameraSim(frontCamera, globalShutterProperties);
                simVision.addCamera(simFrontCam, constants.FRONT_CAMERA_TRANSFORM);

                simFrontCam.enableRawStream(true); // localhost:1181
                simFrontCam.enableProcessedStream(true); // localhost:1182
                simFrontCam.enableDrawWireframe(drawWireframes);
            }
            if (constants.HAS_LEFT_CAMERA) {
                simLeftCam = new PhotonCameraSim(leftCamera, globalShutterProperties);
                simVision.addCamera(simLeftCam, constants.LEFT_CAMERA_TRANSFORM);

                simLeftCam.enableRawStream(true);
                simLeftCam.enableProcessedStream(true);
                simLeftCam.enableDrawWireframe(drawWireframes);
            }
            if (constants.HAS_RIGHT_CAMERA) {
                simRightCam = new PhotonCameraSim(rightCamera, globalShutterProperties);
                simVision.addCamera(simRightCam, constants.RIGHT_CAMERA_TRANSFORM);

                simRightCam.enableRawStream(true);
                simRightCam.enableProcessedStream(true);
                simRightCam.enableDrawWireframe(drawWireframes);
            }
            if (constants.HAS_REAR_CAMERA) {
                simRearCam = new PhotonCameraSim(rearCamera, globalShutterProperties);
                simVision.addCamera(simRearCam, constants.REAR_CAMERA_TRANSFORM);

                simRearCam.enableRawStream(true);
                simRearCam.enableProcessedStream(true);
                simRearCam.enableDrawWireframe(drawWireframes);
            }
        }
    }

    @Override
    public void updateState() {
        currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (tempSimBool) {
            if (robotState.getDrivePose() != null) {
                simVision.update(robotState.getDrivePose());
            }
        }

        if (constants.HAS_FRONT_CAMERA) {
            frontResult = frontCamera.getLatestResult();
            frontSampleTime = frontResult.getTimestampSeconds();
            
            if (frontResult.hasTargets()) {
                frontBestTarget = frontResult.getBestTarget();
                frontCamEstimation = PhotonUtils.estimateFieldToRobotAprilTag(frontBestTarget.getBestCameraToTarget(), 
                                                                            tags.getTagPose(frontBestTarget.getFiducialId()).get(), 
                                                                            constants.FRONT_CAMERA_TRANSFORM).toPose2d();

                frontCameraPub.set(new double[] {
                    frontCamEstimation.getX(),
                    frontCamEstimation.getY(),
                    frontCamEstimation.getRotation().getDegrees()
                });
            } else {
                frontCamEstimation = null;
                // frontCameraPub.set(null);
            }
        }
        if (constants.HAS_LEFT_CAMERA) {
            leftResult = leftCamera.getLatestResult();
            leftSampleTime = leftResult.getTimestampSeconds();

            if (leftResult.hasTargets()) {
                leftBestTarget = leftResult.getBestTarget();
                leftCamEstimation = PhotonUtils.estimateFieldToRobotAprilTag(leftBestTarget.getBestCameraToTarget(),
                                                                        tags.getTagPose(leftBestTarget.getFiducialId()).get(),
                                                                        constants.LEFT_CAMERA_TRANSFORM).toPose2d();
                
                leftCameraPub.set(new double[] {
                    leftCamEstimation.getX(),
                    leftCamEstimation.getY(),
                    leftCamEstimation.getRotation().getDegrees()
                });
            } else {
                leftCamEstimation = null;
                // leftCameraPub.set(null);
            }
        }
        if (constants.HAS_RIGHT_CAMERA) {
            rightResult = rightCamera.getLatestResult();
            rightSampleTime = rightResult.getTimestampSeconds();

            if (rightResult.hasTargets()) {
                rightBestTarget = rightResult.getBestTarget();
                rightCamEstimation = PhotonUtils.estimateFieldToRobotAprilTag(rightBestTarget.getBestCameraToTarget(),
                                                                            tags.getTagPose(rightBestTarget.getFiducialId()).get(),
                                                                            constants.RIGHT_CAMERA_TRANSFORM).toPose2d();

                rightCameraPub.set(new double[] {
                    rightCamEstimation.getX(),
                    rightCamEstimation.getY(),
                    rightCamEstimation.getRotation().getDegrees()
                });
            } else {
                rightCamEstimation = null;
                // rightCameraPub.set(null);
            }
        }
        if (constants.HAS_REAR_CAMERA) {
            rearResult = rearCamera.getLatestResult();
            rearSampleTime = rearResult.getTimestampSeconds();

            if (rearResult.hasTargets()) {
                rearBestTarget = rearResult.getBestTarget();
                rearCamEstimation = PhotonUtils.estimateFieldToRobotAprilTag(rearBestTarget.getBestCameraToTarget(),
                                                                            tags.getTagPose(rearBestTarget.getFiducialId()).get(),
                                                                            constants.REAR_CAMERA_TRANSFORM).toPose2d();
                
                rearCameraPub.set(new double[] {
                        rearCamEstimation.getX(),
                        rearCamEstimation.getY(),
                        rearCamEstimation.getRotation().getDegrees()
                });
            } else {
                rearCamEstimation = null;
                // rearCameraPub.set(null);
            }
        }
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
