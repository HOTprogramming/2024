package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.Camera.*;

import java.io.IOException;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

public class Camera implements SubsystemBase {
    RobotState robotState;

    PhotonCamera frontCamera;
    PhotonCamera rearCamera;

    PhotonPipelineResult frontResult;
    PhotonPipelineResult rearResult;

    AprilTagFieldLayout tags;

    VisionSystemSim simVision;
    PhotonCameraSim simFrontCam;
    PhotonCameraSim simRearCam;
    SimCameraProperties globalShutterProperties;
    
    // docs https://docs.photonvision.org/ 

    public Camera(RobotState robotState) {
        this.robotState = robotState;
        
        try {
            tags = AprilTagFieldLayout.loadFromResource(("/org/Apriltags/2024-crescendo.json"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // C:\Users\Public\wpilib\2024\maven\edu\wpi\first\apriltag

        frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
        rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

        if (Utils.isSimulation()) {
            globalShutterProperties = new SimCameraProperties();
            globalShutterProperties.setFPS(60);
            globalShutterProperties.setCalibration(1920, 1080, Rotation2d.fromDegrees(130));

            simFrontCam = new PhotonCameraSim(frontCamera, globalShutterProperties);
            simRearCam = new PhotonCameraSim(rearCamera, globalShutterProperties);

            simVision = new VisionSystemSim("SimVision");
            simVision.addCamera(simFrontCam, new Transform3d(FRONT_CAMERA_REALITIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION));
            simVision.addCamera(simRearCam, new Transform3d(REAR_CAMERA_REALITIVE_POSITION, REAR_CAMERA_RELATIVE_ROTATION));

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
        if (Utils.isSimulation()) {
            simVision.update(robotState.getDrivePose());
            

        }
        // frontResult = frontCamera.getLatestResult();
        // rearResult = rearCamera.getLatestResult();
        
        // frontResult.getBestTarget().getBestCameraToTarget();
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
