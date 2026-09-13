package frc.robot.utils.Camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class Camera {
  // The intrinsics and distortion coefficients are only actually used for sim. If only used for
  // technically dont need CameraConstants for real robot
  public record CameraConstants(
      String name,
      Transform3d robotToCamera, // probably gonna need a deeper understanding of transform 3d
      Matrix<N3, N3> intrinsicsMatrix, // 3x3 matrix that translates points in 3d to 2d points?
      Matrix<N8, N1>
          distCoeffs) {} // distortion? what is the point of doing this, and why is it an 8x1
}
