package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitUntilCommand extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  private final double m_timeStamp;

  public double startTime;

  static public double startOffset = 0.75;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public WaitUntilCommand(double seconds) {
    m_timeStamp = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed((m_timeStamp - startTime) - startOffset);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> (m_timeStamp - startTime) - startOffset, null);
  }
}
