package frc.robot.components.battery;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
