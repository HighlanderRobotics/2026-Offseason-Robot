// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;


public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    private IndexerIO indexerIO; // make a io object
    private IndexerIoInputsAutoLogged indexerInputs = new IndexerIoInputsAutoLogged(); // this will erorr until the subsystem is finished

    // adding alerts
    private final Alert indexerDisconnectedAlert = new Alert("Indexer disconnected", AlertType.kError);
    private final Alert kickerLeaderDisconnectedAlert = new Alert("Kicker leader disconnected", AlertType.kError);
    private final Alert kickerFollowerDisconnectedAlert = new Alert("Kicker follower disconnected", AlertType.kError);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
