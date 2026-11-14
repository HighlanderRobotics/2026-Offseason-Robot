// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import frc.robot.Robot;
import frc.robot.Robot.RobotMode;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;


public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */

  private IndexerIO indexerIO; // make a io object
  private IndexerIoInputsAutoLogged indexerInputs = new IndexerIoInputsAutoLogged(); // this will erorr until the subsystem is finished

  // adding alerts
  private final Alert indexerDisconnectedAlert = new Alert("Indexer disconnected", AlertType.kError);
  private final Alert kickerLeaderDisconnectedAlert = new Alert("Kicker leader disconnected", AlertType.kError);
  private final Alert kickerFollowerDisconnectedAlert = new Alert("Kicker follower disconnected", AlertType.kError); 

  private SysIdRoutine indexerSysid = new SysIdRoutine(
    new Config(
      null, // ramp rate
      null, // step voltage
      null, // timeout
      (state) -> Logger.recordOutput("Indexer/SysID State", state.toString())),
    new Mechanism((volts) -> indexerIO.setIndexerVoltage(volts.in(Volts)), null, this));

  private SysIdRoutine kickerSysid = new SysIdRoutine(
    new Config(
      null,
      null,
      null,
      (state) -> Logger.recordOutput("Kicker/SysID State", state.toString())),
    new Mechanism((volts) -> indexerIO.setKickerVoltage(volts.in(Volts)), null, this));

  public IndexerSubsystem(CANBus canBus) {
    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      this.indexerIO = new IndexerIOSim(canBus);
    } else {
      this.indexerIO = new IndexerIO(canBus);
    }
  }

  /*
  Commands:
  Index
  Kick
  reverse
  stop
   */
  
  // i had to look at the indexersubsystem file in the main branch for voltage numbers
  public Command index() {
    return this.run(() -> {
      indexerIO.setIndexerVoltage(3.0);
      indexerIO.setKickerVoltage(-4.0);
    });
  }

  public Command kick() {
    return this.run(() -> {
      indexerIO.setIndexerVoltage(5.0);
      indexerIO.setKickerVoltage(10.0);
    });
  }

  public Command reverse() {
    return this.run(() -> {
      indexerIO.setIndexerVoltage(-5.0);
      indexerIO.setKickerVoltage(-5.0);
    });
  }

  // idk what the preshoot command is but i saw it in the other sections of the code so i added it
  public Command preshoot() {
    return this.run(() -> {
      indexerIO.setIndexerVoltage(-3.0);
      indexerIO.setKickerVoltage(-4.0);
    });
  }

  public Command rest() {
    return this.run(() -> {
      indexerIO.setIndexerVoltage(0.0);
      indexerIO.setKickerVoltage(0.0);
    });
  }

  public Command runIndexerSysID() {
    return Commands.sequence(
      indexerSysid.quasistatic(Direction.kForward),
      indexerSysid.quasistatic(Direction.kReverse),
      indexerSysid.dynamic(Direction.kForward),
      indexerSysid.dynamic(Direction.kReverse)
    );
  }

  public Command runKickerSysID() {
    return Commands.sequence(
      kickerSysid.quasistatic(Direction.kForward),
      kickerSysid.quasistatic(Direction.kReverse),
      kickerSysid.dynamic(Direction.kForward),
      kickerSysid.dynamic(Direction.kReverse)
    );
  }
  

  @Override
  public void periodic() {
    indexerIO.updateinputs(indexerInputs);
    Logger.processInputs("Indexer", indexerInputs);

    indexerDisconnectedAlert.set(!indexerInputs.indexerConnected);
    kickerLeaderDisconnectedAlert.set(!indexerInputs.kickerLeaderConnected);
    kickerFollowerDisconnectedAlert.set(!indexerInputs.kickerFollowerConnected);
  }
}
