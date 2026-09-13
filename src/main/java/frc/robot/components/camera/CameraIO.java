package frc.robot.components.camera;

import frc.robot.components.camera.Camera.CameraConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public boolean stale = true;
    // stale != connected
    public boolean connected = false;
  }

  public void updateInputs(CameraIOInputs inputs);

  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult);

  public String getName();

  public CameraConstants getCameraConstants();
}
