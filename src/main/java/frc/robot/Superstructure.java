package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.SlapdownSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.TrenchPoses;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import frc.robot.utils.autoaim.ShotTrees;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum SuperState {
    IDLE,
    INTAKE,
    SCORE,
    FEED,
    SCORE_FLOW,
    FEED_FLOW,
    SPIN_UP_SCORE,
    SPIN_UP_FEED,
    SPIN_UP_SCORE_FLOW,
    SPIN_UP_FEED_FLOW,
    DEFENSE,
    SPIT;

    private final Trigger stateTrigger;

    private SuperState() {
      stateTrigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return stateTrigger;
    }

    public void bindCommands(Command... commands) {
      stateTrigger.whileTrue(Commands.parallel(commands));
    }

    public boolean isASpinUpState() {
      return this.equals(SPIN_UP_FEED)
          || this.equals(SPIN_UP_SCORE)
          || this.equals(SPIN_UP_FEED_FLOW)
          || this.equals(SPIN_UP_SCORE_FLOW);
    }
  }

  public enum FeedTarget {
    LEFT,
    RIGHT
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

  private boolean shouldFeed = false;
  private boolean shouldFlow = false;
  private boolean defense = false;

  private FeedTarget feedTarget = FeedTarget.LEFT;

  private Trigger intakeReq;
  private Trigger scoreReq;
  private Trigger feedReq;
  private Trigger flowReq;
  private Trigger shooterReady;
  private Trigger defenseReq = new Trigger(() -> defense);
  private Trigger spitReq;

  private final IndexerSubsystem indexer;
  private final DrumSubsystem drum;
  private final SlapdownSubsystem intake;

  private final Supplier<Pose2d> robotPoseSupplier;

  public Superstructure(
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator,
      IndexerSubsystem indexer,
      DrumSubsystem drum,
      SlapdownSubsystem intake,
      Supplier<Pose2d> robotPoseSupplier) {
    this.indexer = indexer;
    this.drum = drum;
    this.intake = intake;

    this.robotPoseSupplier = robotPoseSupplier;

    // NOTE! MUST BE CALLED IN THIS ORDER!
    addRequests(driver, operator);
    bindTransitions();
    bindCommands();
  }

  // Must be called by robot sim periodic
  public void simulationPeriodic() {
    // Logs in sim
    Logger.recordOutput("Superstructure/Requests/Intake", intakeReq);
    Logger.recordOutput("Superstructure/Requests/Score", scoreReq);
    Logger.recordOutput("Superstructure/Requests/Feed", feedReq);
    Logger.recordOutput("Superstructure/Requests/Flow", flowReq);
    Logger.recordOutput("Superstructure/Should Feed", shouldFeed);
    Logger.recordOutput("Superstructure/Should Flow", shouldFlow);
    Logger.recordOutput("Superstructure/Defense", defenseReq);
    Logger.recordOutput("Superstructure/Shooter Ready", shooterReady);
    Logger.recordOutput("Superstructure/Feed Target", feedTarget);
    Logger.recordOutput("Superstructure/Is a spin up state", state.isASpinUpState());
  }

  private void addRequests(
      CommandXboxControllerSubsystem driver, CommandXboxControllerSubsystem operator) {
    intakeReq = driver.leftTrigger();
    scoreReq = driver.rightTrigger().and(() -> !shouldFeed).and(() -> !isNearTrench());
    feedReq = driver.rightTrigger().and(() -> shouldFeed).and(() -> !isNearTrench());
    flowReq = new Trigger(() -> shouldFlow);
    shooterReady = new Trigger(drum::readyToShoot).debounce(0.25);
    spitReq = driver.povDown();

    // TODO: MAKE SINGULAR CONTROLLER MODE
    operator.povUp().onTrue(Commands.runOnce(() -> defense = true));
    operator.povDown().onTrue(Commands.runOnce(() -> defense = false));

    operator.a().onTrue(Commands.runOnce(() -> shouldFlow = true));
    operator.b().onTrue(Commands.runOnce(() -> shouldFlow = false));

    operator.x().onTrue(Commands.runOnce(() -> shouldFeed = false));
    operator.y().onTrue(Commands.runOnce(() -> shouldFeed = true));

    operator.leftBumper().onTrue(Commands.runOnce(() -> feedTarget = FeedTarget.LEFT));
    operator.rightBumper().onTrue(Commands.runOnce(() -> feedTarget = FeedTarget.RIGHT));
  }

  private void bindTransitions() {
    bindTransition(SuperState.IDLE, intakeReq, SuperState.INTAKE);
    bindTransition(SuperState.INTAKE, intakeReq.negate(), SuperState.IDLE);

    bindTransition(SuperState.IDLE, scoreReq.and(flowReq.negate()), SuperState.SPIN_UP_SCORE);
    bindTransition(SuperState.IDLE, scoreReq.and(flowReq), SuperState.SPIN_UP_SCORE_FLOW);
    bindTransition(SuperState.SPIN_UP_SCORE, scoreReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_SCORE_FLOW, scoreReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_SCORE, shooterReady.and(flowReq.negate()), SuperState.SCORE);
    bindTransition(SuperState.SPIN_UP_SCORE_FLOW, shooterReady.and(flowReq), SuperState.SCORE_FLOW);
    bindTransition(SuperState.SCORE, flowReq, SuperState.SCORE_FLOW);
    bindTransition(SuperState.SCORE_FLOW, flowReq.negate(), SuperState.SCORE);
    bindTransition(SuperState.SCORE, scoreReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SCORE_FLOW, scoreReq.negate(), SuperState.IDLE);

    bindTransition(SuperState.IDLE, feedReq.and(flowReq.negate()), SuperState.SPIN_UP_FEED);
    bindTransition(SuperState.IDLE, feedReq.and(flowReq), SuperState.SPIN_UP_FEED_FLOW);
    bindTransition(SuperState.SPIN_UP_FEED, feedReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_FEED_FLOW, feedReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.SPIN_UP_FEED, shooterReady.and(flowReq.negate()), SuperState.FEED);
    bindTransition(SuperState.SPIN_UP_FEED_FLOW, shooterReady.and(flowReq), SuperState.FEED_FLOW);
    bindTransition(SuperState.FEED, flowReq, SuperState.FEED_FLOW);
    bindTransition(SuperState.FEED_FLOW, flowReq.negate(), SuperState.FEED);
    bindTransition(SuperState.FEED, feedReq.negate(), SuperState.IDLE);
    bindTransition(SuperState.FEED_FLOW, feedReq.negate(), SuperState.IDLE);

    // Any to defense
    defenseReq.onTrue(Commands.runOnce(() -> state = SuperState.DEFENSE));
    bindTransition(SuperState.DEFENSE, defenseReq.negate(), SuperState.IDLE);

    spitReq.onTrue(Commands.runOnce(() -> state = SuperState.SPIT));
    bindTransition(SuperState.SPIT, spitReq.negate(), SuperState.IDLE);
  }

  // TODO other mechs
  private void bindCommands() {
    SuperState.IDLE.bindCommands(indexer.rest(), intake.restExtended(), drum.rest());

    SuperState.INTAKE.bindCommands(
        indexer.rest(), // Should we index?
        intake.intake(),
        drum.rest());

    SuperState.SPIN_UP_SCORE.bindCommands(
        indexer.rest(), intake.restExtended(), drum.shoot(this::getHubShotData));

    SuperState.SPIN_UP_SCORE_FLOW.bindCommands(
        indexer.rest(), intake.intake(), drum.shoot(this::getHubShotData));

    SuperState.SCORE.bindCommands(
        indexer.kick(), intake.restExtended(), drum.shoot(this::getHubShotData));

    SuperState.SCORE_FLOW.bindCommands(
        indexer.kick(), intake.intake(), drum.shoot(this::getHubShotData));

    SuperState.SPIN_UP_FEED.bindCommands(indexer.rest(), intake.restExtended(), drum.testShot());

    SuperState.SPIN_UP_FEED_FLOW.bindCommands(
        indexer.rest(), intake.intake(), drum.shoot(this::getFeedShotData));

    SuperState.FEED.bindCommands(indexer.kick(), intake.restExtended(), drum.testShot());

    SuperState.FEED_FLOW.bindCommands(
        indexer.kick(), intake.intake(), drum.shoot(this::getFeedShotData));

    SuperState.DEFENSE.bindCommands(indexer.rest(), intake.restRetracted(), drum.rest());

    SuperState.SPIT.bindCommands(indexer.reverse(), intake.outtake(), drum.spit());
  }

  public static SuperState getState() {
    return state;
  }

  private void bindTransition(SuperState start, Trigger transitionTrigger, SuperState end) {
    start.getTrigger().and(transitionTrigger).onTrue(Commands.runOnce(() -> state = end));
  }

  private ShotData getHubShotData() {
    return ShotTrees.HUB_SHOT_TREE.calculateShot(robotPoseSupplier.get());
  }

  private ShotData getFeedShotData() {
    Translation2d feedTargetPos = FieldUtils.FeedTargets.getFeedTarget(feedTarget).getTranslation();
    return ShotTrees.FEED_SHOT_TREE.calculateShot(robotPoseSupplier.get(), feedTargetPos);
  }

  @AutoLogOutput(key = "Swerve/Near Trench")
  public boolean isNearTrench() {
    double x = robotPoseSupplier.get().getX();
    double y = robotPoseSupplier.get().getY();

    boolean inXTol =
        MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getX(), x, 2)
            || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getX(), x, 2);
    boolean inYTol =
        MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getY(), y, 0.515)
            || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getY(), y, 0.515);
    return inXTol && inYTol;
  }
}
