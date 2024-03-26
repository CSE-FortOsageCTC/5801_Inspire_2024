package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArmSubsystem;

public class AmpArmCommand extends Command{
    AmpArmSubsystem ampArmSubsystem;
    PIDController ampArmPID;
    boolean isUp;

    public AmpArmCommand(){
        ampArmSubsystem = AmpArmSubsystem.getInstance();
        ampArmPID = new PIDController(0, 0, 0);
        ampArmPID.setSetpoint(0);
        SmartDashboard.putNumber("Amp Arm P", 0);
        SmartDashboard.putNumber("Amp Arm I", 0);
        SmartDashboard.putNumber("Amp Arm D", 0);
        isUp = false;
        addRequirements(ampArmSubsystem);
    }

    @Override
    public void initialize(){
        isUp = !isUp;
    }

    @Override
    public void execute(){
        ampArmPID.setP(SmartDashboard.getNumber("Amp Arm P", 0));
        ampArmPID.setI(SmartDashboard.getNumber("Amp Arm I", 0));
        ampArmPID.setD(SmartDashboard.getNumber("Amp Arm D", 0));

        double speed;

        if (isUp){
            ampArmPID.setSetpoint(0);//TODO change this to the encoder value of the amp arm when it is up
            speed = ampArmPID.calculate(ampArmPID.getSetpoint() - ampArmSubsystem.getEncoderValue());
           
        }
        else{
            ampArmPID.setSetpoint(0);
            speed = ampArmPID.calculate(ampArmSubsystem.getEncoderValue());
        }

        ampArmSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean end){
        ampArmSubsystem.setSpeed(0);
        ampArmPID.reset();
    }

    @Override
    public boolean isFinished(){
        if (isUp){
            return ampArmSubsystem.getEncoderValue() >= ampArmPID.getSetpoint();
        }
        else{
            return ampArmSubsystem.getEncoderValue() <= ampArmPID.getSetpoint();
    }
    }
}
