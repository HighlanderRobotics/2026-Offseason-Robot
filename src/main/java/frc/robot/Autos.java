package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autos.Action;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final AutoFactory factory;
  private static boolean autoIntake;
  private static boolean autoScore;

  @AutoLogOutput(key = "Superstructure/Auto Intake Request")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  public enum Action {
    NOTHING,
    DELAYED_SCORE,
    INTAKE
  }

  // testing please organize later :)
  public enum Path {
    // testing
    LStartToFirstDip("LTStartToFirstDip", Action.NOTHING),
    LDipToFirstShoot("LTDipToFirstShootBump", Action.NOTHING),
    LSecondDip("LTSecondDip", Action.NOTHING);

    private final String name;
    private final Action action;

    /**
     * @param name the name of the path in choreo. MUST match
     * @param action the action to perform during/at the end of the path
     */
    private Path(String name, Action action) {
      this.name = name;
      this.action = action;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(name);
    }
  }

  public Autos(SwerveSubsystem swerve) {
    this.swerve = swerve;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
              Logger.recordOutput("Choreo/Active Traj Name", traj.name());
            });
  }

  public Command runPath(Path path, AutoRoutine routine) {
    Action action = path.action;
    switch (action) {
      case DELAYED_SCORE:
        return delayedScorePath(path, routine);
      case INTAKE:
        return intakeScorePath(path, routine);
      case NOTHING:
        return emptyPath(path, routine);
      default:
        return Commands.none();
    }
  }

  public Command emptyPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAllReqsFalse(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command delayedScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        stopIntaking(),
        startScoring(),
        Commands.waitSeconds(3));
  }

  public Command intakeScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        startIntaking(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command shootPreload() {
    return Commands.sequence(startScoring(), swerve.stop().repeatedly().withTimeout(3));
  }

  public Command startIntaking() {
    return Commands.runOnce(() -> autoIntake = true);
  }

  public Command stopIntaking() {
    return Commands.runOnce(() -> autoIntake = false);
  }

  public Command startScoring() {
    return Commands.runOnce(() -> autoScore = true);
  }

  public Command stopScoring() {
    return Commands.runOnce(() -> autoScore = false);
  }

  public Command setAllReqsFalse() {
    return Commands.sequence(stopIntaking(), stopScoring());
  }

  public void setAllReqsFalsenotcmd() {
    autoIntake = false;
    autoScore = false;
  }

  public Command createAuto(String name, Path[] paths, Command... startingCommands) {
    final AutoRoutine routine = factory.newRoutine(name);

    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry().andThen(startingCommands);
    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }
    routine.active().onTrue(autoCommand);
    return routine.cmd();
  }

  public Command getTesting() {
    return createAuto("Single dip auto", new Path[] {Path.LStartToFirstDip, Path.LDipToFirstShoot});
  }

  public Command getTestDoubleDipAuto() {
    return createAuto(
        "Single dip auto",
        new Path[] {
          Path.LStartToFirstDip, Path.LDipToFirstShoot, Path.LSecondDip, Path.LDipToFirstShoot
        });
  }
}
