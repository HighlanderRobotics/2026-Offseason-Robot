package frc.robot.superstructure.subsystem;

import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;

public interface Subsystem {

  /**
   * Runs once per loop cycle
   *
   * @param state the current superstructure state
   * @param robotSnapshot a snapshot of the robot's sensors and inputs at this loop cycle
   */
  void periodic(SuperState state, RobotSnapshot robotSnapshot);

  /**
   * Runs once per loop cycle in simulation. Does not run on real robot
   *
   * @param state the current superstructure state
   * @param robotSnapshot a snapshot of the robot's sensors and inputs at this loop cycle
   */
  void simulationPeriodic(SuperState state, RobotSnapshot robotSnapshot);
}
