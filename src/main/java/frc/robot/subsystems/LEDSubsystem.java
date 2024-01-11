package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  private final Spark blinkin;

  public LEDSubsystem() {
    blinkin = new Spark(3);
  }

  public void SetLEDs (double color) {
    blinkin.set(color);
  }
}