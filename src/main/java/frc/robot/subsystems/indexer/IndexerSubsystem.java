package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import frc.robot.Robot;
import frc.robot.Robot.RobotMode;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  private final Alert indexerDisconnectedAlert =
      new Alert("Disconnected flywheel leader!", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Disconnected flywheel follower!", AlertType.kError);

  private SysIdRoutine indexerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/SysID State", state.toString())),
          new Mechanism((volts) -> indexerIO.setIndexerVoltage(volts.in(Volts)), null, this));

  private SysIdRoutine kickerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Kicker/SysID State", state.toString())),
          new Mechanism((volts) -> indexerIO.setKickerVoltage(volts.in(Volts)), null, this));

  public IndexerSubsystem(CANBus canbus) {
    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      this.indexerIO = new IndexerIOSim(canbus);
    } else {
      this.indexerIO = new IndexerIO(canbus);
    }
  }

  // Kick ball out of indexer
  public Command kick() {
    return this.run(
        () -> {
          indexerIO.setIndexerVoltage(10);
          indexerIO.setKickerVoltage(-7);
        });
  }

  // Rotate indexer
  public Command index() {
    return this.run(
        () -> {
          indexerIO.setIndexerVoltage(7);
          indexerIO.setKickerVoltage(5.5);
        });
  }

  // Sends balls backwards if gets stuck
  public Command reverse() {
    return this.run(
        () -> {
          indexerIO.setIndexerVoltage(-5);
          indexerIO.setKickerVoltage(-5);
        });
  }

  // Rest
  public Command rest() {
    return this.run(
        () -> {
          indexerIO.setIndexerVoltage(0);
          indexerIO.setKickerVoltage(0);
        });
  }

  public Command runIndexerSysid() {
    return Commands.sequence(
        indexerSysid.quasistatic(Direction.kForward),
        indexerSysid.quasistatic(Direction.kReverse),
        indexerSysid.dynamic(Direction.kForward),
        indexerSysid.dynamic(Direction.kReverse));
  }

  public Command runKickerSysid() {
    return Commands.sequence(
        kickerSysid.quasistatic(Direction.kForward),
        kickerSysid.quasistatic(Direction.kReverse),
        kickerSysid.dynamic(Direction.kForward),
        kickerSysid.dynamic(Direction.kReverse));
  }

  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Indexer", indexerInputs);

    indexerDisconnectedAlert.set(!indexerInputs.indexerConnected);
    kickerDisconnectedAlert.set(!indexerInputs.kickerConnected);
  }
}
