package frc.robot.subsystems.drum;

import com.ctre.phoenix6.CANBus;
import frc.robot.superstructure.util.RobotSnapshot;
import frc.robot.superstructure.util.SuperState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DrumSubsystemTest {
  DrumSubsystem drum;

  @BeforeEach
  void setup() {
    drum = new DrumSubsystem(new CANBus());
  }

  @Test
  void periodicStateCheckIsExhaustive() {
    for (SuperState state : SuperState.values()) {
      drum.periodic(state, new RobotSnapshot());
    }
  }
}
