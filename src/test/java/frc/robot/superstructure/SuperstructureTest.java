package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureTest {
  Superstructure superstructure;

  @BeforeEach
  void setup() {
    // No subsystems for now
    superstructure = new Superstructure();
  }

  @Test
  void intakeOnLeftTrigger() {
    // Verify initial condition
    assertEquals(SuperState.IDLE, superstructure.getState());

    RobotSnapshot leftTriggerPressed =
        new RobotSnapshot(true, false, false, false, false, false, false, false, Pose2d.kZero);

    // Handle transition
    superstructure.periodic(leftTriggerPressed);

    // Should be intaking now
    assertEquals(SuperState.INTAKE, superstructure.getState());
  }

  @Test
  void intakeToIdleWhenTriggerReleased() {
    superstructure.setStateForTesting(SuperState.INTAKE);

    // Verify initial condition
    assertEquals(SuperState.INTAKE, superstructure.getState());

    RobotSnapshot nothingPressed = new RobotSnapshot();

    superstructure.periodic(nothingPressed);

    // Should be in IDLE now
    assertEquals(SuperState.IDLE, superstructure.getState());
  }

  @Test
  void spinUpScoreToIdleWhenNothingPressed() {
    superstructure.setStateForTesting(SuperState.SPIN_UP_SCORE);

    // Verify initial condition
    assertEquals(SuperState.SPIN_UP_SCORE, superstructure.getState());

    RobotSnapshot nothingPressed = new RobotSnapshot();

    superstructure.periodic(nothingPressed);

    // Should be in IDLE now
    assertEquals(SuperState.IDLE, superstructure.getState());
  }

  @Test
  void spinUpScoreToIdleWhenNothingPressedWithShooterReady() {
    superstructure.setStateForTesting(SuperState.SPIN_UP_SCORE);

    // Verify initial condition
    assertEquals(SuperState.SPIN_UP_SCORE, superstructure.getState());

    RobotSnapshot drumReady =
        new RobotSnapshot(false, false, false, false, false, true, false, false, Pose2d.kZero);

    superstructure.periodic(drumReady);

    // Should be in IDLE now
    assertEquals(SuperState.IDLE, superstructure.getState());
  }

  @Test
  void idleToSpinUpScoreOnRightTrigger() {
    // Verify initial condition
    assertEquals(SuperState.IDLE, superstructure.getState());

    RobotSnapshot rightTriggerPressed =
        new RobotSnapshot(false, true, false, false, false, false, false, false, Pose2d.kZero);

    // Handle transition
    superstructure.periodic(rightTriggerPressed);

    // Should be intaking now
    assertEquals(SuperState.SPIN_UP_SCORE, superstructure.getState());
  }

}
