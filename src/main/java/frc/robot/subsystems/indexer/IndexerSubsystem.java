package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.Robot.RobotMode;
import frc.robot.superstructure.subsystem.Subsystem;
import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem implements Subsystem {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  private final Alert indexerDisconnectedAlert =
      new Alert("Disconnected indexer!", AlertType.kError);
  private final Alert kickerLeaderDisconnectedAlert =
      new Alert("Disconnected kicker leader!", AlertType.kError);
  private final Alert kickerFollowerDisconnectedAlert =
      new Alert("Disconnected kicker follower!", AlertType.kError);

  public IndexerSubsystem(CANBus canbus) {
    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      this.indexerIO = new IndexerIOSim(canbus);
    } else {
      this.indexerIO = new IndexerIO(canbus);
    }
  }

  @Override
  public void periodic(SuperState state, RobotSnapshot robotSnapshot) {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Indexer", indexerInputs);

    indexerDisconnectedAlert.set(!indexerInputs.indexerConnected);
    kickerLeaderDisconnectedAlert.set(!indexerInputs.kickerLeaderConnected);
    kickerFollowerDisconnectedAlert.set(!indexerInputs.kickerFollowerConnected);

    // Still not sure the giant switch statement is the best solution to this
    switch (state) {
      case IDLE:
      case DEFENSE:
        rest();
        break;

      case INTAKE:
        index();
        break;

      case SPIN_UP_FEED:
      case SPIN_UP_FEED_FLOW:
      case SPIN_UP_SCORE:
      case SPIN_UP_SCORE_FLOW:
        preshoot();
        break;

      case SCORE:
      case SCORE_FLOW:
      case FEED:
      case FEED_FLOW:
        kick();
        break;

      case SPIT:
        reverse();
        break;

      default:
        // Should never hit. Verified exhaustivity in a unit test
        throw new UnsupportedOperationException(
            "Indexer doesn't have an action bound to " + state.toString());
    }
  }

  @Override
  public void simulationPeriodic(SuperState state, RobotSnapshot robotSnapshot) {}

  private void rest() {
    indexerIO.setIndexerVoltage(0.0);
    indexerIO.setKickerVoltage(0.0);
  }

  private void index() {
    indexerIO.setIndexerVoltage(3.0);
    indexerIO.setKickerVoltage(-4.0);
  }

  private void preshoot() {
    indexerIO.setIndexerVoltage(-3.0);
    indexerIO.setKickerVoltage(-4.0);
  }

  private void kick() {
    indexerIO.setIndexerVoltage(5.0);
    indexerIO.setKickerVoltage(10.0);
  }

  private void reverse() {
    indexerIO.setIndexerVoltage(-5.0);
    indexerIO.setKickerVoltage(-5.0);
  }
}
