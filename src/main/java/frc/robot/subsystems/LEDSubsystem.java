package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  //private final Spark blinkin;
  private final PWM pwm;
  private static LEDSubsystem ledSubsystem;
  public double ledCycle = 0;

  private boolean isClimbing = false;

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

  public void setClimbing(boolean isClimbing) {
    this.isClimbing = isClimbing;
  }

  public void setLEDs (double color) {
    //blinkin.set(color);
    pwm.setPulseTimeMicroseconds(2125);
    pwm.setSpeed(0.99);
    pwm.setSpeed(color);
  }

  public void increment() {
    if (ledCycle != 1) {
      ledCycle += .01;
      System.out.println("Hi guys!");
    }
  }

  public void decrement() {
    if (ledCycle != -1) {
      ledCycle -= .01;
    }
    
  }

  public void setDefaultLEDs () {
    if (isClimbing) {
      ledSubsystem.setLEDs(-0.87);
    }
    else {
      ledSubsystem.setLEDs(0.99);
    }
  }
}