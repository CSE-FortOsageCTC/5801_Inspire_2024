package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpArmSubsystem extends SubsystemBase{
    private static AmpArmSubsystem ampArmSubsystem;
    public CANSparkMax ampArmMotor;
    public double highLimit = 40;
    public double lowLimit = 0;
    public boolean isUp = false;

    public static AmpArmSubsystem getInstance(){
        if(ampArmSubsystem == null){
            ampArmSubsystem = new AmpArmSubsystem();
        }
        return ampArmSubsystem;
    }

    private AmpArmSubsystem(){
        ampArmMotor = new CANSparkMax(30, MotorType.kBrushless);

        ampArmMotor.setIdleMode(IdleMode.kCoast);
        ampArmMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        ampArmMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        ampArmMotor.enableVoltageCompensation(10);
        ampArmMotor.setSmartCurrentLimit(20);
        ampArmMotor.burnFlash();
    }

    public void setSpeed(double speed){
        if ((ampArmMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed() && speed > 0) || 
        (ampArmMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed() && speed < 0)) {
            ampArmMotor.set(0);
        } else {
            // if (speed > 0 || ampArmMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            //     isUp = true;
            // } else {
            //     isUp = false;
            // }

            ampArmMotor.set(speed);
        }
    }

    public double getEncoderValue(){
        return ampArmMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        if (ampArmMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()){
            highLimit = ampArmMotor.getEncoder().getPosition();
            // ampArmMotor.getEncoder().setPosition(0);//TODO change this to the encoder value when amp arm is up
        }
        if (ampArmMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()){
            lowLimit = ampArmMotor.getEncoder().getPosition();
        }
        SmartDashboard.putNumber("Amp Arm Encoder", getEncoderValue());
    }
}
