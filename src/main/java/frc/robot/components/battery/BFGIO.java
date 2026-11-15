package frc.robot.components.battery;

import com.playingwithfusion.BattFuelGauge;
import com.playingwithfusion.BattFuelGauge.BatteryManufacturer;
import org.littletonrobotics.junction.AutoLog;

public class BFGIO {
  @AutoLog
  public static class BFGIOInputs {
    public boolean isConnected = false;

    public double voltage = 0.0;
    public double dischargeCurrent = 0.0;
    public double charge = 0.0;
    // Maybe more stuff to log need to check

  }

  private BattFuelGauge bfg;

  public BFGIO(int id) {
    this.bfg = new BattFuelGauge(id);
  }

  public void updateInputs(BFGIOInputs inputs) {
    inputs.isConnected = bfg.isConnected();

    inputs.voltage = bfg.getVoltage();
    inputs.dischargeCurrent = bfg.getCurrent();
    inputs.charge = bfg.getRemainingChargePct();
  }

  public void saveLog() {
    // Use default logfile naming
    bfg.saveLog("");
  }

  public String getName() {
    return bfg.getNickname();
  }

  public long getSerialNumber() {
    return bfg.getSerialNumber();
  }

  public BatteryManufacturer getManufacturer() {
    return bfg.getManufacturer();
  }
}
