package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorSubsystem;

public class AngleShooterUtil {
    
    private final ElevatorSubsystem s_ElevatorSubsystem;

    private static double m_encoderDiff;
    private final PIDController upPidController;
    private final PIDController downPidController;

    public AngleShooterUtil(double encoderDiff) {

        s_ElevatorSubsystem = ElevatorSubsystem.getInstance();
        AngleShooterUtil.m_encoderDiff = encoderDiff;

        this.upPidController = new PIDController(0, 0, 0);
        this.downPidController = new PIDController(0, 0, 0);

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
   }

   public void initialize() {
    upPidController.reset();
    downPidController.reset();
   }

   public void reset() {
    upPidController.reset();
    downPidController.reset();
   }


   public double calculateElevatorSpeed () {

    
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

    double headingError = this.m_encoderDiff;
    SmartDashboard.putNumber("Heading Error", headingError);
    double feedForward = 0.5;
    double speed = 0;

    if (m_encoderDiff < 0) {
        speed = upPidController.calculate(m_encoderDiff, 0);

    }
    else {
        speed = downPidController.calculate(m_encoderDiff, 0);
    }
    speed = MathUtil.clamp(speed, -.3, .3);
    SmartDashboard.putNumber(("Elevator Speed"), speed);

    if (Math.abs(headingError) > 30) {
        return (headingError < 0) ? feedForward : -feedForward;
    } else {
        return speed;
    }
   }

public static void updateTargetDiff(double angle) {

    m_encoderDiff = angle;

   }

   public boolean isFinished () {
    return upPidController.atSetpoint() && downPidController.atSetpoint();
   }

   public void end() {
    System.out.println("it ended");
    upPidController.reset();
    downPidController.reset();
   }

}