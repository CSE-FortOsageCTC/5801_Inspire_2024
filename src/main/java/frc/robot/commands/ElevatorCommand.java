package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private double setPoint;
    private AngleShooterUtil angleShooterUtil;
    public ElevatorCommand(double setPoint){
        this.setPoint = setPoint;
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
    double elevatorValue = elevatorSubsystem.getElevatorValue();
    angleShooterUtil.updateTargetDiff(elevatorValue - setPoint);
<<<<<<< HEAD
    elevatorSubsystem.setElevatorSpeed(angleShooterUtil.calculateElevatorSpeed());
=======
    shooterSubsystem.setElevatorSpeed(setPoint);
>>>>>>> main

    }

    @Override
    public void end(boolean end){
        elevatorSubsystem.setElevatorSpeed(0);
        angleShooterUtil.reset();

    }
}