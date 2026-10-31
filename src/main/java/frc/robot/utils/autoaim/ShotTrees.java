package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class ShotTrees {
  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  static {
    HUB_SHOT_TREE.put(1.87, new ShotData(Rotation2d.fromDegrees(10), 40.0 - 4, 1.31));
    HUB_SHOT_TREE.put(2.38, new ShotData(Rotation2d.fromDegrees(13), 42.0 - 4, 1.28));
    HUB_SHOT_TREE.put(2.62, new ShotData(Rotation2d.fromDegrees(16), 42.0 - 4, 1.18));
    HUB_SHOT_TREE.put(3.39, new ShotData(Rotation2d.fromDegrees(18), 46.0 - 4, 1.3));
    HUB_SHOT_TREE.put(3.67, new ShotData(Rotation2d.fromDegrees(22), 46.0 - 4, 1.29));
    HUB_SHOT_TREE.put(3.86, new ShotData(Rotation2d.fromDegrees(24), 48.0 - 4, 1.24));
    HUB_SHOT_TREE.put(4.27, new ShotData(Rotation2d.fromDegrees(26), 48.0 - 4, 1.45));
    HUB_SHOT_TREE.put(4.65, new ShotData(Rotation2d.fromDegrees(27), 48.0 - 4, 1.38));
    HUB_SHOT_TREE.put(5.25, new ShotData(Rotation2d.fromDegrees(28), 50.0 - 4, 1.44));
  }

  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static {
    FEED_SHOT_TREE.put(4.94, new ShotData(Rotation2d.fromDegrees(35), 30.0, 1.15));
    FEED_SHOT_TREE.put(5.5, new ShotData(Rotation2d.fromDegrees(35), 35.0, 0.98));
    FEED_SHOT_TREE.put(6.08, new ShotData(Rotation2d.fromDegrees(40), 40.0, 1.02));
    FEED_SHOT_TREE.put(6.87, new ShotData(Rotation2d.fromDegrees(45), 45.0, 1.12));
    FEED_SHOT_TREE.put(7.29, new ShotData(Rotation2d.fromDegrees(45), 47.0, 1.1));
    FEED_SHOT_TREE.put(7.98, new ShotData(Rotation2d.fromDegrees(45), 48.0, 1.05));
    FEED_SHOT_TREE.put(8.45, new ShotData(Rotation2d.fromDegrees(45), 50.0, 1.13));
    FEED_SHOT_TREE.put(9.05, new ShotData(Rotation2d.fromDegrees(45), 52.0, 1.4));
    FEED_SHOT_TREE.put(9.9, new ShotData(Rotation2d.fromDegrees(45), 55.0, 1.5));
  }
}
