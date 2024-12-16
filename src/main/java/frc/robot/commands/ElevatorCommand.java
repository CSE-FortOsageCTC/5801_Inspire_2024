package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private double speed;
    private AngleShooterUtil angleShooterUtil;

    public ElevatorCommand(double speed){
        this.speed = speed;
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }

    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }

    @Override
    public void execute(){
        elevatorSubsystem.setElevatorSpeed(speed);
    }

    @Override
    public void end(boolean end){
        elevatorSubsystem.setElevatorSpeed(0);
        angleShooterUtil.reset();

    }
}