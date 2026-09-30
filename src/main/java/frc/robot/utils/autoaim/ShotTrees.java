package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class ShotTrees {
    public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();
    static {
        // TODO: POPULATE AND REMOVE THIS
        HUB_SHOT_TREE.put(0.0, new ShotData(new Rotation2d(), 0.0, 0.0));
    }

    public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();
    static {
        // TODO: POPULATE AND REMOVE THIS
        FEED_SHOT_TREE.put(0.0, new ShotData(new Rotation2d(), 0.0, 0.0));
    }
}
