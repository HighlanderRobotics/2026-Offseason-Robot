package frc.robot.components.battery;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BFGSubsystem extends SubsystemBase {

  BFGIO io;
  BFGIOInputsAutoLogged bfgInputs = new BFGIOInputsAutoLogged();

  public BFGSubsystem(int id) {
    this.io = new BFGIO(id);
  }

  @Override
  public void periodic() {
    io.updateInputs(bfgInputs);
    Logger.processInputs("Battery", bfgInputs);
  }
}
