package frc.robot.superstructure;

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

/** Holds the state machine graph and manages all transitions between states */
public class Superstructure {

  // TODO: SHOULD THIS JS BE A VALUE GRAPH?
  private final ImmutableNetwork<SuperState, TransitionCondition> stateGraph;

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
}
