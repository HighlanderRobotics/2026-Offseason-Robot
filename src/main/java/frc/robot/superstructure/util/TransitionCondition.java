package frc.robot.superstructure.util;

@FunctionalInterface
public interface TransitionCondition {

  /**
   * Returns if a transition should occur, given the passed in snapshot
   *
   * @param snapshot
   * @return
   */
  boolean test(RobotSnapshot snapshot);
}
