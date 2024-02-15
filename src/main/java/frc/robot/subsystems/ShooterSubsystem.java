package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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

    public double getElevatorValue(){
        return rightElevator.getEncoder().getPosition();
    }

    public void setElevatorSpeed(double speed){
        double elevatorValue = getElevatorValue();
        if (elevatorValue < Constants.Swerve.minElevatorValue && speed < 0){
            speed = 0;
        }

        else if (elevatorValue > Constants.Swerve.maxElevatorValue && speed > 0){
            speed = 0;
        }
        rightElevator.set(speed);


        SmartDashboard.putNumber("Encoder Value", rightElevator.getEncoder().getPosition());

    }
    public void spinKicker(double percent){
        kicker.set(ControlMode.PercentOutput, percent);
    }
   
}
