package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

/**
 * calculates a dumper shot while the robot is moving
 *
 * <p>robot's current speeds are used to project its pose forward by the shot's time of flight. Then
 * shot table is used at the projected distance, and the returned heading points the dumper at the
 * target from that projected position.
 */
public final class AutoAim {
  private static final int MAX_ITERATIONS = 12;
  private static final double DISTANCE_CONVERGENCE_METERS = 1.0e-3;

  public record ShotSolution(
      ShotData shotData,
      Rotation2d targetHeading,
      Pose2d projectedRobotPose,
      double projectedDistanceMeters) {}

  private static Pose2d projectPose(
      Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, double timeSeconds) {
    return robotPose.exp(
        new Twist2d(
            robotRelativeSpeeds.vxMetersPerSecond * timeSeconds,
            robotRelativeSpeeds.vyMetersPerSecond * timeSeconds,
            robotRelativeSpeeds.omegaRadiansPerSecond * timeSeconds));
  }

  /**
   * @param robotPose current field-relative robot pose
   * @param robotRelativeSpeeds current chassis speeds
   * @param target field-relative target position
   * @param shotTree distance-to-shot lookup table
   * @return shot data and the field-relative robot heading to align to
   */
  public static ShotSolution calculateShot(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d target,
      InterpolatingShotTree shotTree) {
    if (robotPose == null || robotRelativeSpeeds == null || target == null || shotTree == null) {
      throw new IllegalArgumentException("Pose, speeds, target, and shot tree can't be empty");
    }

    double timeOfFlightSeconds = 0.0;
    double distanceMeters = target.getDistance(robotPose.getTranslation());
    Pose2d projectedPose = robotPose;
    ShotData shotData = shotTree.get(distanceMeters);

    // The flight time depends on distance, and the distance depends on how far the robot travels
    // during flight repeat until the distance/time estimate settles.
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlightSeconds = shotData.timeOfFlightSecs();
      projectedPose = projectPose(robotPose, robotRelativeSpeeds, timeOfFlightSeconds);

      double updatedDistanceMeters = target.getDistance(projectedPose.getTranslation());
      ShotData updatedShotData = shotTree.get(updatedDistanceMeters);
      double distanceChangeMeters = Math.abs(updatedDistanceMeters - distanceMeters);

      distanceMeters = updatedDistanceMeters;
      shotData = updatedShotData;
      if (distanceChangeMeters <= DISTANCE_CONVERGENCE_METERS) {
        timeOfFlightSeconds = shotData.timeOfFlightSecs();
        projectedPose = projectPose(robotPose, robotRelativeSpeeds, timeOfFlightSeconds);
        distanceMeters = target.getDistance(projectedPose.getTranslation());
        shotData = shotTree.get(distanceMeters);
        break;
      }
    }

    Translation2d projectedRobotToTarget = target.minus(projectedPose.getTranslation());
    Rotation2d targetHeading = projectedRobotToTarget.getAngle();

    Logger.recordOutput("AutoAimOnMove/ProjectedRobotPose", projectedPose);
    Logger.recordOutput("AutoAimOnMove/ProjectedDistanceMeters", distanceMeters);
    Logger.recordOutput("AutoAimOnMove/TimeOfFlightSeconds", timeOfFlightSeconds);
    Logger.recordOutput("AutoAimOnMove/TargetHeading", targetHeading);

    return new ShotSolution(shotData, targetHeading, projectedPose, distanceMeters);
  }
}
