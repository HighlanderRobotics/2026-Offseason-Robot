package frc.robot.utils.Camera;

import frc.robot.utils.Camera.Camera.CameraConstants;
import org.littletonrobotics.junction.AutoLog;
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

  public String getName();

  public CameraConstants getCameraConstants();
}
