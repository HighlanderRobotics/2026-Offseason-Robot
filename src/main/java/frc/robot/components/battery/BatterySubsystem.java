package frc.robot.components.battery;

import com.playingwithfusion.BattFuelGauge.BatteryManufacturer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BatterySubsystem extends SubsystemBase {

  BFGIO io;
  BFGIOInputsAutoLogged bfgInputs = new BFGIOInputsAutoLogged();

  public BatterySubsystem(int id) {
    this.io = new BFGIO(id);
  }

  @Override
  public void periodic() {
    io.updateInputs(bfgInputs);
    Logger.processInputs("Battery", bfgInputs);
  }

  // IO Passthrough
  public void saveLog() {
    io.saveLog();
  }

  public String getBatteryName() {
    return io.getName();
  }

  public long getSerialNumber() {
    return io.getSerialNumber();
  }

  public BatteryManufacturer getManufacturer() {
    return io.getManufacturer();
  }
}
