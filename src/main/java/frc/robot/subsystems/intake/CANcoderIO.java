package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public class CANcoderIO {
  @AutoLog
  public static class CANcoderIOInputs {
    public boolean connected = false;
    public Rotation2d cancoderPositionRotations = new Rotation2d();
    public StatusCode status;
  }

  protected final CANcoder cancoder;

  private final StatusSignal<Angle> cancoderAbsolutePositionRotations;

  public CANcoderIO(int cancoderID, CANcoderConfiguration config, CANBus canbus) {
    cancoder = new CANcoder(cancoderID, canbus);
    cancoderAbsolutePositionRotations = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, cancoderAbsolutePositionRotations);
    cancoder.getConfigurator().apply(config);
    cancoder.optimizeBusUtilization();
  }

  public void updateInputs(CANcoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(cancoderAbsolutePositionRotations);
    inputs.connected = BaseStatusSignal.isAllGood(cancoderAbsolutePositionRotations);
    inputs.status = cancoderAbsolutePositionRotations.getStatus();
    inputs.cancoderPositionRotations =
        Rotation2d.fromRotations(cancoderAbsolutePositionRotations.getValueAsDouble());
  }
}
