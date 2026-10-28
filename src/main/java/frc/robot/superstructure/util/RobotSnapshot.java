package frc.robot.superstructure.util;

import edu.wpi.first.math.geometry.Pose2d;

public record RobotSnapshot(
    boolean leftTrigger,
    boolean rightTrigger,
    boolean shouldFeed,
    boolean isNearTrench,
    boolean shouldFlow,
    boolean drumReady,
    boolean spitRequest,
    boolean defenseRequest,
    Pose2d robotPose) {

  // For some tests
  public RobotSnapshot() {
    this(false, false, false, false, false, false, false, false, Pose2d.kZero);
  }

  public boolean scoreRequest() {
    return rightTrigger() && !shouldFeed() && !isNearTrench();
  }

  public boolean feedRequest() {
    return rightTrigger() && shouldFeed() && !isNearTrench();
  }
}
