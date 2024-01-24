package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.Camera.*;
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
    
    

    public Camera(RobotState robotState) {
        this.robotState = robotState;
        
        tags = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2024-crescendo.json");
        // C:\Users\Public\wpilib\2024\maven\edu\wpi\first\apriltag

        frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
        rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

        if (Utils.isSimulation()) {
            simFrontCam = new PhotonCameraSim(frontCamera);

            simVision = new VisionSystemSim("SimVision");
            simVision.addCamera(simFrontCam, new Transform3d(FRONT_CAMERA_REALITIVE_POSITION, FRONT_CAMERA_RELATIVE_ROTATION));
            
        }
    }

    @Override
    public void updateState() {
        frontResult = frontCamera.getLatestResult();
        rearResult = rearCamera.getLatestResult();
        
        frontResult.getBestTarget().getBestCameraToTarget();
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
