package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.ShooterSubsystem;

public class ElevatorCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double setPoint;
    private AngleShooterUtil angleShooterUtil;
    public ElevatorCommand(double setPoint){
        this.setPoint = setPoint;
        shooterSubsystem = ShooterSubsystem.getInstance();
        addRequirements(shooterSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }

    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }

    @Override
    public void execute(){
    double elevatorValue = shooterSubsystem.getElevatorValue();
    angleShooterUtil.updateTargetDiff(elevatorValue - setPoint);
    shooterSubsystem.setElevatorSpeed(angleShooterUtil.calculateElevatorSpeed());

    }

    @Override
    public void end(boolean end){
        shooterSubsystem.setElevatorSpeed(0);
        angleShooterUtil.reset();

    }
}