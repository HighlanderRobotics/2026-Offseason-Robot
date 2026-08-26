package frc.robot.utils;

import org.ironmaple.simulation.SimulatedArena;

/** Maple sim arena with no obsticles */
public class EvergreenArena extends SimulatedArena {

  public EvergreenArena() {
    super(new FieldMap() {});
  }

  @Override
  public void placeGamePiecesOnField() {}
}
