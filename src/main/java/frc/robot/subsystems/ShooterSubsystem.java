package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;

    private CANSparkMax topShooter;
    private CANSparkMax bottomShooter;
    private CANSparkMax shooterElevator;
    private VictorSPX kicker;
    private PIDController elevatorPID;

    private DoubleSolenoid shooterSolenoid;


    public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
    private ShooterSubsystem(){
        topShooter = new CANSparkMax(20, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(21, MotorType.kBrushless);
        shooterElevator = new CANSparkMax(25, MotorType.kBrushless);

        bottomShooter.follow(topShooter, true);
        topShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        bottomShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        topShooter.burnFlash();
        bottomShooter.burnFlash();
        // leftElevator.follow(rightElevator);
        elevatorPID = new PIDController(0, 0, 0);
        shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);

        shooterElevator.getForwardLimitSwitch(Type.kNormallyClosed);
        shooterElevator.getReverseLimitSwitch(Type.kNormallyClosed);
        shooterElevator.burnFlash();
        
    }
    public void setKicker(){
        shooterSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void resetKicker(){
        shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public double getElevatorValue(){
        return shooterElevator.getEncoder().getPosition();
    }

    public void setElevatorSpeed(double speed){
        // double elevatorValue = getElevatorValue();
        // if (elevatorValue < Constants.Swerve.minElevatorValue && speed < 0){
        //     speed = 0;
        // }

        // else if (elevatorValue > Constants.Swerve.maxElevatorValue && speed > 0){
        //     speed = 0;
        // }
        shooterElevator.set(speed);


        SmartDashboard.putNumber("Encoder Value", shooterElevator.getEncoder().getPosition());

    }
    public void setFlyWheels(double percent){
        topShooter.set(percent);
    }
   
}
