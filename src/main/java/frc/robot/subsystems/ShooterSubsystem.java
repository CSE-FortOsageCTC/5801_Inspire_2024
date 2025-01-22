package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;
    private static LEDSubsystem ledSubsystem;

    private SparkMax topShooter;
    private SparkMax bottomShooter;

    private SparkMaxConfig configurations;

    private LimitSwitchConfig limitSwitchConfig;

    private DoubleSolenoid shooterSolenoid;



    public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }

    private ShooterSubsystem(){
        topShooter = new SparkMax(20, MotorType.kBrushless);
        bottomShooter = new SparkMax(21, MotorType.kBrushless);

        limitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

        configurations.idleMode(IdleMode.kBrake);
        configurations.follow(topShooter);
        configurations.voltageCompensation(10);
        configurations.apply(limitSwitchConfig);
        
        bottomShooter.configure(configurations, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        topShooter.configure(configurations, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);

        ledSubsystem = LEDSubsystem.getInstance();

        //SmartDashboard.putNumber("Shooter Percent Multiplier", 0.4);
        
    }
    public void setKicker(){
        shooterSolenoid.set(DoubleSolenoid.Value.kForward);
        ledSubsystem.setLEDs(-0.57); //"fire large"

    }

    public void resetKicker(){
        shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void setFlyWheels(double percent){
        topShooter.set(percent);
        //topShooter.set(percent * SmartDashboard.getNumber("Shooter Percent Multiplier", 0.4));
    }
   
}
