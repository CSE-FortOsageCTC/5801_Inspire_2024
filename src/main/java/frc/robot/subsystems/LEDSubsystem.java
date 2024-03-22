package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  //private final Spark blinkin;
  private final PWM pwm;
  private static LEDSubsystem ledSubsystem;

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

  private LEDSubsystem() {
    //blinkin = new Spark(1);
    pwm = new PWM(1);
  }

  public void setLEDs (double color) {
    //blinkin.set(color);
    pwm.setPulseTimeMicroseconds(2125);
    pwm.setSpeed(0.99);
    pwm.setSpeed(color);
  }

  public void setDefaultLEDs () {
    ledSubsystem.setLEDs(0.99);
  }
}