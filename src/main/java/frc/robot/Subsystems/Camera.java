package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
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


import java.io.IOException;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import frc.robot.ConstantsFolder.ConstantsBase;

public class Camera implements SubsystemBase {
    ConstantsBase.Camera constants;
    RobotState robotState;

    PhotonCamera frontCamera;
    PhotonCamera rearCamera;

    PhotonPipelineResult frontResult;
    PhotonPipelineResult rearResult;

    double frontSampleTime;
    double rearSampleTime;


    AprilTagFieldLayout tags;

    VisionSystemSim simVision;
    PhotonCameraSim simFrontCam;
    PhotonCameraSim simRearCam;
    SimCameraProperties globalShutterProperties;

    Pose2d frontCamEstimation;
    Pose2d rearCamEstimation;

    Pose2d lastFrontEstimation;
    Pose2d lastRearEstimation;

    Nat<N3> rows = new Nat<N3>() {

        @Override
        public int getNum() {
            return 3;
        }
        
    };
    Nat<N2> colls = new Nat<N2>() {

        @Override
        public int getNum() {
            return 2;
        }
        
    };


    Matrix<N3, N2> stdevs = new Matrix(rows, colls);


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

        // C:\Users\Public\wpilib\2024\maven\edu\wpi\first\apriltag

        frontCamera = new PhotonCamera(constants.FRONT_CAMERA_NAME);

        rearCamera = new PhotonCamera(constants.REAR_CAMERA_NAME);

        if (Utils.isSimulation()) {
            globalShutterProperties = new SimCameraProperties();
            globalShutterProperties.setFPS(60);
            globalShutterProperties.setCalibration(1920, 1080, Rotation2d.fromDegrees(70));

            simFrontCam = new PhotonCameraSim(frontCamera, globalShutterProperties);
            simRearCam = new PhotonCameraSim(rearCamera, globalShutterProperties);

            simVision = new VisionSystemSim("SimVision");
            simVision.addCamera(simFrontCam, new Transform3d(constants.FRONT_CAMERA_REALITIVE_POSITION, constants.FRONT_CAMERA_RELATIVE_ROTATION));
            simVision.addCamera(simRearCam, new Transform3d(constants.REAR_CAMERA_REALITIVE_POSITION, constants.REAR_CAMERA_RELATIVE_ROTATION));

            simVision.addAprilTags(tags);


            simFrontCam.enableRawStream(true); // localhost:1181
            simFrontCam.enableProcessedStream(true); // localhost:1182

            simFrontCam.enableDrawWireframe(true); // resource intesive

            simRearCam.enableRawStream(true); // localhost:1183
            simRearCam.enableProcessedStream(true); // localhost:1184

            simRearCam.enableDrawWireframe(true); // resource intesive
        }
    }

    @Override
    public void updateState() {
        lastFrontEstimation = frontCamEstimation;
        lastRearEstimation = rearCamEstimation;

        currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (Utils.isSimulation()) {
            if (robotState.getDrivePose() != null) {
                simVision.update(robotState.getDrivePose());
            }
        }

        frontResult = frontCamera.getLatestResult();
        rearResult = rearCamera.getLatestResult();

        frontSampleTime = frontResult.getTimestampSeconds();
        rearSampleTime = rearResult.getTimestampSeconds();

        robotState.setVisionTimestamps(new double[] {frontSampleTime, rearSampleTime});

        if (frontResult.getBestTarget() != null) {
            frontCamEstimation = tags.getTagPose(frontResult.getBestTarget().getFiducialId()).get()
                            .transformBy(frontResult.getBestTarget().getBestCameraToTarget().inverse())
                            .transformBy(constants.FRONT_CAMERA_TRANSFROM.inverse())
                            .toPose2d();
        } else {
            frontCamEstimation = null;
        }

        if (rearResult.getBestTarget() != null) {
            rearCamEstimation = tags.getTagPose(rearResult.getBestTarget().getFiducialId()).get()
                            .transformBy(rearResult.getBestTarget().getBestCameraToTarget().inverse())
                            .transformBy(constants.REAR_CAMERA_TRANSFROM.inverse())
                            .toPose2d();
        } else {
            rearCamEstimation = null;
        }

        // stdev
        if (lastFrontEstimation != null && frontCamEstimation != null) {
            double[] last = new double[] {lastFrontEstimation.getX(), lastFrontEstimation.getY(), lastFrontEstimation.getRotation().getRadians()};
            double[] current = new double[] {frontCamEstimation.getX(), frontCamEstimation.getY(), frontCamEstimation.getRotation().getRadians()};

            for (int i = 0; i < 3; i++) {
                double avg = (last[i] + current[i]) / 2;
                stdevs.set(i, 0, Math.sqrt(((Math.pow(last[i] - avg, 2)) + (Math.pow(current[i] - avg, 2))) / 2));
            }

        }
       
        if (lastRearEstimation != null && rearCamEstimation != null) {
            double[] last = new double[] {lastRearEstimation.getX(), lastRearEstimation.getY(), lastRearEstimation.getRotation().getRadians()};
            double[] current = new double[] {rearCamEstimation.getX(), rearCamEstimation.getY(), rearCamEstimation.getRotation().getRadians()};

            for (int i = 0; i < 3; i++) {
                double avg = (last[i] + current[i]) / 2;
                stdevs.set(i, 1, Math.sqrt(((Math.pow(last[i] - avg, 2)) + (Math.pow(current[i] - avg, 2))) / 2));
            }

        }

        
        
        robotState.setVisionMeasurements(new Pose2d[] {(frontResult.getBestTarget() != null) ? 
                                                        frontCamEstimation : null,
                                                        (rearResult.getBestTarget() != null) ? 
                                                        rearCamEstimation : null});

        if (frontCamEstimation != null) {
            SmartDashboard.putNumberArray("FrontCamera ODO", new Double[] {frontCamEstimation.getX(), frontCamEstimation.getY(), frontCamEstimation.getRotation().getDegrees()});
        } else {
            SmartDashboard.putNumberArray("FrontCamera ODO", new Double[] {0.0, 0.0, 0.0});
        }

        if (rearCamEstimation != null) {
            SmartDashboard.putNumberArray("RearCamera ODO", new Double[] {rearCamEstimation.getX(), rearCamEstimation.getY(), rearCamEstimation.getRotation().getDegrees()});
        } else {
            SmartDashboard.putNumberArray("RearCamera ODO", new Double[] {0.0, 0.0, 0.0});
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
