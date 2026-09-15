package frc.robot.subsystems.drum;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class DrumSubsystem extends SubsystemBase {
  public static final int FLYWHEEL_LEADER_ID = 0; // TODO: CORRECT ID
  public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: VALUE FROM CAD

  private FlywheelIO flywheelIO;
  private FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();

  private FollowerIO[] followerIOs = new FollowerIO[3];

  private FollowerIOInputsAutoLogged[] followerIOInputs = new FollowerIOInputsAutoLogged[3];

  public DrumSubsystem(CANBus canBus) {
    flywheelIO = new FlywheelIO(canBus);

    // Fill with blank inputs
    Arrays.fill(followerIOInputs, new FollowerIOInputsAutoLogged());

    // TODO: CORRECT VALUES
    followerIOs[0] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus);
    followerIOs[1] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus);
    followerIOs[2] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelIOInputs);
    Logger.processInputs("Drum/Flywheel/Leader", flywheelIOInputs);

    // Update follower inputs
    for (int i = 0; i < followerIOs.length; i++) {
      followerIOs[i].updateInputs(followerIOInputs[i]);
      Logger.processInputs("Drum/Flywheel/Follower " + i, followerIOInputs[i]);
    }
  }
}
