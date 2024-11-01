package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.sql.Driver;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;



public class ChoreoSubsystem extends SubsystemBase{
    
    private Swerve s_Swerve;

    private PIDController autoXPID = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private PIDController autoYPID = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private PIDController autoThetaPID = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

    private static ChoreoSubsystem s_ChoreoSubsystem;

    private ChoreoTrajectory trajectory;
    
    public static ChoreoSubsystem getInstance() {
        if (s_ChoreoSubsystem == null) {
            s_ChoreoSubsystem = new ChoreoSubsystem();
        }
        return s_ChoreoSubsystem;
    }

    public ChoreoSubsystem() {

        s_Swerve = Swerve.getInstance();

    }

    public boolean getFlipped() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        // System.out.println(alliance);
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    private Pose2d getPose() {
        //s_Swerve.setPose(trajectory.getInitialPose());
        // System.out.println(s_Swerve.getPose().getRotation().toString());
        return s_Swerve.getPose();
    }


    public Command setupAutonomousChoreoPath(ChoreoTrajectory traj) {
        // Pose2d initialPose = getFlipped() ? traj.getFlippedInitialPose() : traj.getInitialPose();
        // SmartDashboard.putNumber("Initial Pose X", initialPose.getX());
        // SmartDashboard.putNumber("Initial Pose Y", initialPose.getY());

        // // initialPose = initialPose.rotateBy(Rotation2d.fromDegrees(180));
        // //trajectory = traj;  ??

        // s_Swerve.setPose(initialPose);
        return Choreo.choreoSwerveCommand(
            traj,
            this::getPose,
            autoXPID,
            autoYPID,
            autoThetaPID,
            (ChassisSpeeds speeds) -> s_Swerve.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond, false, true),
            this::getFlipped,
            s_Swerve
        );
    }



}
