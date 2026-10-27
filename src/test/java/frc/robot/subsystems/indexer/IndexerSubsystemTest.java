package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IndexerSubsystemTest {

  IndexerSubsystem indexer;

  @BeforeEach
  void setup() {
    indexer = new IndexerSubsystem(new CANBus());
  }

  @Test
  void periodicStateCheckIsExhaustive() {
    for (SuperState state : SuperState.values()) {
      indexer.periodic(state, new RobotSnapshot());
    }
  }
}
