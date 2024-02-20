package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ElevatorCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final PIDController upPidController;
    private final PIDController downPidController;
    private double setPoint;
    public ElevatorCommand(double setPoint){
        this.setPoint = setPoint;
        this.upPidController = new PIDController(0, 0, 0);
        this.downPidController = new PIDController(0, 0, 0);
        shooterSubsystem = ShooterSubsystem.getInstance();

        upPidController.setTolerance(5);
        upPidController.setSetpoint(300);
        SmartDashboard.putNumber("Up P", 0);
        SmartDashboard.putNumber("Up I", 0);
        SmartDashboard.putNumber("Up D", 0);

        downPidController.setTolerance(5);
        downPidController.setSetpoint(20);
        SmartDashboard.putNumber("Down P", 0);
        SmartDashboard.putNumber("Down I", 0);
        SmartDashboard.putNumber("Down D", 0);
        addRequirements(shooterSubsystem);
    }

    // public void updateSetPoint(double setPoint){
    //     this.setPoint = setPoint;
    //     setPoint = upPidController.getSetpoint();
    // }

    @Override
    public void initialize() {
        upPidController.reset();
        downPidController.reset();
    }

    @Override
    public void execute(){


    double upP = SmartDashboard.getNumber("Up P", 0.0);
    double upI = SmartDashboard.getNumber("Up I", 0.0);
    double upD = SmartDashboard.getNumber("Up D", 0.0);

    double downP = SmartDashboard.getNumber("Down P", 0.0);
    double downI = SmartDashboard.getNumber("Down I", 0.0);
    double downD = SmartDashboard.getNumber("Down D", 0.0);

    this.upPidController.setP(upP);
    this.upPidController.setI(upI);
    this.upPidController.setD(upD);

    this.downPidController.setP(downP);
    this.downPidController.setI(downI);
    this.downPidController.setD(downD);

    double elevatorValue = shooterSubsystem.getElevatorValue();
    double speed = 0;

    if  (elevatorValue < upPidController.getSetpoint()) {
        speed = upPidController.calculate(shooterSubsystem.getElevatorValue(), setPoint);

    }
    else {
        speed = downPidController.calculate(shooterSubsystem.getElevatorValue(), setPoint);
    }
    speed = MathUtil.clamp(speed, -.3, .3);
    SmartDashboard.putNumber(("Elevator Speed"), speed);
    shooterSubsystem.setElevatorSpeed(speed);

    }

    @Override
    public void end(boolean end){
        shooterSubsystem.setElevatorSpeed(0);
        upPidController.reset();
        downPidController.reset();

    }
}