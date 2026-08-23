package frc.robot.utils.Camera;

import frc.robot.utils.Camera.Camera.CameraConstants;
import org.photonvision.PhotonCamera;

public class CameraIOReal implements CameraIO {
  private final CameraConstants constants;
  public PhotonCamera camera;

  public CameraIOReal(CameraConstants constants) {
    this.constants = constants;
    this.camera = new PhotonCamera(constants.name());
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = camera.isConnected();
    var results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      inputs.result = results.get(results.size() - 1);
      inputs.stale = false;
    } else {
      inputs.stale = true;
    }
  }

  @Override
  public String getName() {
    return constants.name();
  }

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }
}
