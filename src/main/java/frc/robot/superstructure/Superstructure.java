package frc.robot.superstructure;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableNetwork;
import com.google.common.graph.NetworkBuilder;
import frc.robot.Robot;
import frc.robot.superstructure.subsystem.Subsystem;
import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;
import frc.robot.superstructure.util.TransitionCondition;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

/** Holds the state machine graph and manages all transitions between states */
public class Superstructure {

  // TODO: SHOULD THIS JS BE A VALUE GRAPH?
  private final ImmutableNetwork<SuperState, TransitionCondition> stateGraph;

  @AutoLogOutput(key = "Superstructure/State")
  private SuperState currentState = SuperState.IDLE;

  private Set<Subsystem> subsystems;

  public Superstructure(Subsystem... subsystems) {
    stateGraph =
        NetworkBuilder.directed()
            .allowsParallelEdges(true) // Maybe should be false?
            .allowsSelfLoops(false)
            .edgeOrder(ElementOrder.insertion())
            .<SuperState, TransitionCondition>immutable()
            .addEdge(SuperState.IDLE, SuperState.INTAKE, (ss) -> ss.leftTrigger())
            .addEdge(SuperState.INTAKE, SuperState.IDLE, (ss) -> !ss.leftTrigger())
            .addEdge(
                SuperState.IDLE,
                SuperState.SPIN_UP_SCORE,
                (ss) -> ss.scoreRequest() && !ss.shouldFlow())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.IDLE, (ss) -> !ss.scoreRequest())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.SCORE, (ss) -> ss.drumReady())
            .addEdge(
                SuperState.SPIN_UP_SCORE, SuperState.SPIN_UP_SCORE_FLOW, (ss) -> ss.shouldFlow())
            .addEdge(SuperState.SCORE, SuperState.IDLE, (ss) -> !ss.scoreRequest())
            .addEdge(SuperState.SCORE, SuperState.SCORE_FLOW, (ss) -> ss.shouldFlow())
            .addEdge(
                SuperState.IDLE,
                SuperState.SPIN_UP_SCORE_FLOW,
                (ss) -> ss.scoreRequest() && ss.shouldFlow())
            .addEdge(SuperState.SPIN_UP_SCORE_FLOW, SuperState.IDLE, (ss) -> !ss.scoreRequest())
            .addEdge(SuperState.SPIN_UP_SCORE_FLOW, SuperState.SCORE_FLOW, (ss) -> ss.drumReady())
            .addEdge(
                SuperState.SPIN_UP_SCORE_FLOW, SuperState.SPIN_UP_SCORE, (ss) -> !ss.shouldFlow())
            .addEdge(SuperState.SCORE_FLOW, SuperState.IDLE, (ss) -> !ss.scoreRequest())
            .addEdge(SuperState.SCORE_FLOW, SuperState.SCORE, (ss) -> !ss.shouldFlow())
            .addEdge(
                SuperState.IDLE,
                SuperState.SPIN_UP_FEED,
                (ss) -> ss.feedRequest() && !ss.shouldFlow())
            .addEdge(SuperState.SPIN_UP_FEED, SuperState.IDLE, (ss) -> !ss.feedRequest())
            .addEdge(SuperState.SPIN_UP_FEED, SuperState.FEED, (ss) -> ss.drumReady())
            .addEdge(SuperState.SPIN_UP_FEED, SuperState.SPIN_UP_FEED_FLOW, (ss) -> ss.shouldFlow())
            .addEdge(SuperState.FEED, SuperState.IDLE, (ss) -> !ss.feedRequest())
            .addEdge(SuperState.FEED, SuperState.FEED_FLOW, (ss) -> ss.shouldFlow())
            .addEdge(
                SuperState.IDLE,
                SuperState.SPIN_UP_FEED_FLOW,
                (ss) -> ss.feedRequest() && ss.shouldFlow())
            .addEdge(SuperState.SPIN_UP_FEED_FLOW, SuperState.IDLE, (ss) -> !ss.feedRequest())
            .addEdge(SuperState.SPIN_UP_FEED_FLOW, SuperState.FEED_FLOW, (ss) -> ss.drumReady())
            .addEdge(
                SuperState.SPIN_UP_FEED_FLOW, SuperState.SPIN_UP_FEED, (ss) -> !ss.shouldFlow())
            .addEdge(SuperState.FEED_FLOW, SuperState.IDLE, (ss) -> !ss.feedRequest())
            .addEdge(SuperState.FEED_FLOW, SuperState.FEED, (ss) -> !ss.shouldFlow())
            .addEdge(SuperState.IDLE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.INTAKE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SCORE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.FEED, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SCORE_FLOW, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.FEED_FLOW, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIN_UP_SCORE_FLOW, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIN_UP_FEED, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIN_UP_FEED_FLOW, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.DEFENSE, SuperState.SPIT, (ss) -> ss.spitRequest())
            .addEdge(SuperState.SPIT, SuperState.IDLE, (ss) -> !ss.spitRequest())
            .addEdge(SuperState.IDLE, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.INTAKE, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SCORE, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.FEED, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SCORE_FLOW, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.FEED_FLOW, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIN_UP_SCORE_FLOW, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIN_UP_FEED, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIN_UP_FEED_FLOW, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIN_UP_SCORE, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.SPIT, SuperState.DEFENSE, (ss) -> ss.defenseRequest())
            .addEdge(SuperState.DEFENSE, SuperState.IDLE, (ss) -> !ss.defenseRequest())
            .build();

    // Linked hash set because we want to preserve the insertion order
    this.subsystems = new LinkedHashSet<Subsystem>(Arrays.asList(subsystems));
  }

  public void periodic(RobotSnapshot snapshot) {
    handleTransitions(snapshot);

    for (Subsystem subsystem : subsystems) {
      if (Robot.isSimulation()) {
        subsystem.simulationPeriodic(currentState, snapshot);
      }

      subsystem.periodic(currentState, snapshot);
    }
  }

  private void handleTransitions(RobotSnapshot snapshot) {
    // Go through graph and transition if needed
    Set<SuperState> successors = stateGraph.successors(currentState);

    boolean foundTransition = false;
    for (SuperState successor : successors) {
      Set<TransitionCondition> edgesToSuccessor =
          stateGraph.edgesConnecting(currentState, successor);
      // This should never hit, because successors by definition have edges, but just in case
      if (edgesToSuccessor.isEmpty()) continue;

      for (TransitionCondition edge : edgesToSuccessor) {
        if (edge.test(snapshot)) {
          // Should transition
          foundTransition = true;
          System.out.println("Changing state to " + successor);
          currentState = successor;
          break; // Only one transition
        }
      }

      if (foundTransition) break; // Make sure to stop looping after a transition is found
    }
  }

  public SuperState getState() {
    return currentState;
  }

  /**
   * <b>For testing ONLY!</b> <br>
   * Changes the state to the passed in state
   *
   * @param newState
   */
  @VisibleForTesting
  void setStateForTesting(SuperState newState) {
    this.currentState = newState;
  }
}
