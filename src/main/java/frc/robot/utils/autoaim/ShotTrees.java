package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class ShotTrees {
  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  static {
    // TODO: POPULATE AND REMOVE THIS
    HUB_SHOT_TREE.put(1.718, new ShotData(Rotation2d.fromDegrees(10), 31.0, 0.87));
    HUB_SHOT_TREE.put(2.62, new ShotData(Rotation2d.fromDegrees(17), 30.0, 0.75));
    HUB_SHOT_TREE.put(3.39, new ShotData(Rotation2d.fromDegrees(20), 33.0, 0.81));
    HUB_SHOT_TREE.put(3.68, new ShotData(Rotation2d.fromDegrees(17), 38.0, 1.1));
    HUB_SHOT_TREE.put(3.89, new ShotData(Rotation2d.fromDegrees(19), 38.0, 1.18));
    HUB_SHOT_TREE.put(4.48, new ShotData(Rotation2d.fromDegrees(20), 40.0, 1.23));
    HUB_SHOT_TREE.put(4.65, new ShotData(Rotation2d.fromDegrees(23), 42.0, 1.34));
    HUB_SHOT_TREE.put(5.31, new ShotData(Rotation2d.fromDegrees(25), 44.0, 1.29));
  }

  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static {
    // TODO: POPULATE AND REMOVE THIS
    FEED_SHOT_TREE.put(0.0, new ShotData(Rotation2d.fromDegrees(25), 30.0, 0.0));
  }
}
