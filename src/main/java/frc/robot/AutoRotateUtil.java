package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class AutoRotateUtil {

    private final Swerve s_Swerve;
    private final PIDController pidController;

    private int m_angle;

    public AutoRotateUtil(Swerve s_Swerve, int angle) {
        this.s_Swerve = s_Swerve;
        this.m_angle = angle == 0 ? 360 : angle;

        // SmartDashboard.putNumber("Angle", this.m_angle);
        this.pidController = new PIDController(0, 0, 0);

        pidController.setTolerance(0.675);
        pidController.setSetpoint(this.m_angle);
        SmartDashboard.putNumber("kP", 0.0065);
        SmartDashboard.putNumber("kI", 0.0025);
        SmartDashboard.putNumber("kD", 0.001);
    }

    public void initialize() {
        pidController.reset();
    }

    public double calculateRotationSpeed() {

        double kP = SmartDashboard.getNumber("kP", 0.0);
        double kI = SmartDashboard.getNumber("kI", 0.0);
        double kD = SmartDashboard.getNumber("kD", 0.0);
        this.pidController.setP(kP);
        this.pidController.setI(kI);
        this.pidController.setD(kD);
        double yaw = s_Swerve.gyro.getYaw().getValue();
        double headingError = this.m_angle - yaw;
        if (headingError > 180) {
            headingError -= 360;
        }
        if (headingError < -180) {
            headingError += 360;
        }
        double speed = pidController.calculate(yaw, yaw + headingError);
        SmartDashboard.putNumber("Speed", speed);
        return speed;
    }

    public boolean isFinished() {
        return pidController.atSetpoint();
        // double speed = pidController.calculate(s_Swerve.getYawDouble());
        // return speed < 0.1;
    }

}