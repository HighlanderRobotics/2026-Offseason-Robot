package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  public static final int INDEXER_ROLLER_ID = 11;
  public static final int KICKER_LEADER_ID = 12;
  public static final int KICKER_FOLLOWER_ID = 13;

  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  
  public IndexerIO io;

  public IndexerSubsystem(CANBus canBus) {
    this.io = new IndexerIO(canBus);

  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      io.updateInputs(inputs);
      Logger.processInputs("Indexer", inputs);

  }

  public Command setIndexerVoltageCommand(DoubleSupplier volts) {
    return this.run(() -> io.setIndexerVoltage(volts.getAsDouble()));
  }
  public Command setKickerVoltageCommand(DoubleSupplier volts) {
    return this.run(() -> io.setKickerVoltage(volts.getAsDouble()));
  }

  public Command kick() {
    return this.run(
        () -> {
          io.setIndexerVoltage(5.0);
          io.setKickerVoltage(10.0);
        });
  }

  // Rotate indexer
  public Command index() {
    return this.run(
        () -> {
          io.setIndexerVoltage(3.0);
          io.setKickerVoltage(-4.0);
        });
  }

  public Command preshoot() {
    return this.run(
        () -> {
          io.setIndexerVoltage(-3.0);
          io.setKickerVoltage(-4.0);
        });
  }

  // Sends balls backwards if gets stuck
  public Command reverse() {
    return this.run(
        () -> {
          io.setIndexerVoltage(-5.0);
          io.setKickerVoltage(-5.0);
        });
  }

  // Rest
  public Command rest() {
    return this.run(
        () -> {
          io.setIndexerVoltage(0);
          io.setKickerVoltage(0);
        });
  }



} 

