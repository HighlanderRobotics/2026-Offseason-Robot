package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.CommandXboxControllerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum SuperState {
    IDLE,
    INTAKE,
    SCORE,
    FEED,
    SCORE_FLOW,
    FEED_FLOW,
    SPIN_UP_SCORE,
    SPIN_UP_FEED,
    DEFENSE;

    private final Trigger stateTrigger;

    private SuperState() {
      stateTrigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return stateTrigger;
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

  private boolean shouldFeed = false;
  private boolean shouldFlow = false;
  private boolean defense = false;

  private Trigger intakeReq;
  private Trigger scoreReq;
  private Trigger feedReq;
  private Trigger flowReq;
  private Trigger shooterReady;
  private Trigger defenseReq = new Trigger(() -> defense);

  public Superstructure(
      CommandXboxControllerSubsystem driver, CommandXboxControllerSubsystem operator) {

    // NOTE! MUST BE CALLED IN THIS ORDER!
    addRequests(driver, operator);
    bindTransitions();
    bindCommands();
  }

  // Must be called by robot sim periodic
  public void simulationPeriodic() {
    // Logs in sim
    Logger.recordOutput("Superstructure/Requests/Intake", intakeReq);
    Logger.recordOutput("Superstructure/Requests/Score", scoreReq);
    Logger.recordOutput("Superstructure/Requests/Feed", feedReq);
    Logger.recordOutput("Superstructure/Requests/Flow", flowReq);
    Logger.recordOutput("Superstructure/Should Feed", shouldFeed);
    Logger.recordOutput("Superstructure/Shooter Ready", shooterReady);
  }

  private void addRequests(
      CommandXboxControllerSubsystem driver, CommandXboxControllerSubsystem operator) {
    intakeReq = driver.leftTrigger();
    scoreReq = driver.rightTrigger().and(() -> !shouldFeed);
    feedReq = driver.rightTrigger().and(() -> shouldFeed);
    flowReq = new Trigger(() -> shouldFlow);

    operator.povUp().onTrue(Commands.runOnce(() -> defense = true));
    operator.povDown().onTrue(Commands.runOnce(() -> defense = false));

    operator.a().onTrue(Commands.runOnce(() -> shouldFlow = true));
    operator.b().onTrue(Commands.runOnce(() -> shouldFlow = true));

    operator.x().onTrue(Commands.runOnce(() -> shouldFeed = false));
    operator.y().onTrue(Commands.runOnce(() -> shouldFeed = true));
  }

  private void bindTransitions() {
    bindTransition(SuperState.IDLE, intakeReq, SuperState.INTAKE);
    bindTransition(SuperState.INTAKE, intakeReq.negate(), SuperState.IDLE);

    bindTransition(SuperState.IDLE, scoreReq, SuperState.SPIN_UP_SCORE);
    bindTransition(SuperState.IDLE, scoreReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_SCORE, shooterReady.and(flowReq.negate()), SuperState.SCORE);
    bindTransition(SuperState.SPIN_UP_SCORE, shooterReady.and(flowReq), SuperState.SCORE_FLOW);
    bindTransition(SuperState.SCORE, scoreReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SCORE_FLOW, scoreReq.negate(), SuperState.IDLE);

    bindTransition(SuperState.IDLE, feedReq, SuperState.SPIN_UP_FEED);
    bindTransition(SuperState.SPIN_UP_FEED, feedReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_FEED, shooterReady.and(flowReq.negate()), SuperState.FEED);
    bindTransition(SuperState.SPIN_UP_FEED, shooterReady.and(flowReq), SuperState.FEED_FLOW);
    bindTransition(SuperState.FEED, feedReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.FEED_FLOW, feedReq.negate(), SuperState.IDLE);

    // Any to defense
    defenseReq.onTrue(Commands.runOnce(() -> state = SuperState.DEFENSE));
    bindTransition(SuperState.DEFENSE, defenseReq.negate(), SuperState.IDLE);
  }

  // TODO
  private void bindCommands() {}

  public static SuperState getState() {
    return state;
  }

  private void bindTransition(SuperState start, Trigger transitionTrigger, SuperState end) {
    start.getTrigger().and(transitionTrigger).onTrue(Commands.runOnce(() -> state = end));
  }
}
