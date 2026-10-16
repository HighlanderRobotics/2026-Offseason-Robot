package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;

public class ModuleIOBlank extends ModuleIOReal {

  public ModuleIOBlank(ModuleConstants moduleConstants, CANBus canbus) {
    super(moduleConstants, canbus);
  }

  public void setDriveVoltage(double volts, boolean withFoc) {}

  public void setDriveVoltage(double volts) {}

  public void setDriveVelocitySetpoint(double setpointMetersPerSecond) {}

  public void setTurnVoltage(double volts) {}

  public void setTurnPositionSetpoint(Rotation2d setpoint) {}
}
