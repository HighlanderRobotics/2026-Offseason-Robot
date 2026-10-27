package frc.robot.superstructure.util;

public record RobotSnapshot(
    boolean leftTrigger,
    boolean rightTrigger,
    boolean shouldFeed,
    boolean isNearTrench,
    boolean shouldFlow,
    boolean drumReady,
    boolean spitRequest,
    boolean defenseRequest) {

  // For some tests
  public RobotSnapshot() {
    this(false, false, false, false, false, false, false, false);
  }

  public boolean scoreRequest() {
    return rightTrigger() && !shouldFeed() && !isNearTrench();
  }

  public boolean feedRequest() {
    return rightTrigger() && shouldFeed() && !isNearTrench();
  }
}
