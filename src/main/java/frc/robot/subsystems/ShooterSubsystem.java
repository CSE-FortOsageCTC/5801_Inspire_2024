package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;

    private CANSparkMax topShooter;
    private CANSparkMax bottomShooter;
    private CANSparkMax rightElevator;
    private CANSparkMax leftElevator;
    private VictorSPX kicker;
    private PIDController elevatorPID;

    public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
    private ShooterSubsystem(){
        topShooter = new CANSparkMax(20, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(21, MotorType.kBrushless);
        rightElevator = new CANSparkMax(25, MotorType.kBrushless);
        leftElevator = new CANSparkMax(26, MotorType.kBrushless);
        kicker = new VictorSPX(19);
        bottomShooter.follow(topShooter);
        leftElevator.follow(rightElevator);
        elevatorPID = new PIDController(0, 0, 0);
        
    }
    public void shoot(double speed){
        topShooter.set(speed);
    }
    public Command setAngle(double angle){
        elevatorPID.setSetpoint(angle);
        return run(()->rightElevator.set(elevatorPID.calculate(angle)));
    }
    public void setElevatorSpeed(double speed){
        rightElevator.set(speed);
        
    }
    public void spinKicker(double percent){
        kicker.set(ControlMode.PercentOutput, percent);
    }
   
}
