package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpArmSubsystem;

public class AmpArmCommand extends Command{
    AmpArmSubsystem ampArmSubsystem;
    boolean isUp;
    Joystick controller;
    int trigger;
    double triggerAxis;
    PIDController ampPIDController;

    public AmpArmCommand(Joystick controller){
        this.controller = controller;
        ampPIDController = new PIDController(0, 0, 0);
        trigger = XboxController.Axis.kLeftTrigger.value;
        ampArmSubsystem = AmpArmSubsystem.getInstance();
        addRequirements(ampArmSubsystem);

        ampPIDController.setP(0.02);
        ampPIDController.setI(0);
        ampPIDController.setD(0);
    }

    @Override
    public void execute(){
        triggerAxis = controller.getRawAxis(trigger);
        if (triggerAxis > Constants.stickDeadband){
            ampArmSubsystem.isUp = true;
            double speed = MathUtil.clamp(ampPIDController.calculate(ampArmSubsystem.getEncoderValue() - (ampArmSubsystem.lowLimit + 42)), -0.15, 0.5);
            ampArmSubsystem.setSpeed(speed);
            // SmartDashboard.putNumber("PID Amp Output", speed);
        } else {
            ampArmSubsystem.isUp = false;
            if (ampArmSubsystem.getEncoderValue() > (ampArmSubsystem.highLimit - 20)) {
                ampArmSubsystem.setSpeed(-0.2);
            } else {
                ampArmSubsystem.setSpeed(0);
            }
        }

        
    }
}
