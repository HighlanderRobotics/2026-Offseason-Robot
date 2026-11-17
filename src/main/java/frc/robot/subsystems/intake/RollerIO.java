// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.stream.BaseStream;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/**
Things to log
velocity rots per second
applied voltage
position rotations
supply current
stator current
temp
connected
 */
public class RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public double velocityRotsPerSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double positionRotations = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean connected = false;
  }

  protected final TalonFX motor;

  private final StatusSignal < AngularVelocity > angularVelocityRotsPerSecond;
  private final StatusSignal < Voltage > appliedVoltage;
  private final StatusSignal < Current > supplyCurrentAmps;
  private final StatusSignal < Current > statorCurrentAmps;
  private final StatusSignal < Temperature > temperatureCelsius;
  private final StatusSignal < Angle > positionRotations;

  private double setpointVelocity;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  public RollerIO(int motorID, TalonFXConfiguration config, CANBus canBus) {
    motor = new TalonFX(motorID, canBus);

    angularVelocityRotsPerSecond = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    statorCurrentAmps = motor.getStatorCurrent();
    supplyCurrentAmps = motor.getSupplyCurrent();
    temperatureCelsius = motor.getDeviceTemp();
    positionRotations = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      angularVelocityRotsPerSecond,
      supplyCurrentAmps,
      statorCurrentAmps,
      appliedVoltage,
      temperatureCelsius,
      positionRotations);
    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
  }
}