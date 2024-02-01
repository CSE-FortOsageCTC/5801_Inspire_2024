package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DefaultTeleopSub extends SubsystemBase{
    


    public Swerve s_Swerve = Swerve.getInstance();
    public SkyLimelight s_Limelight = SkyLimelight.getInstance();

    private static DefaultTeleopSub autoAlignSwerve;

    public static DefaultTeleopSub getInstance() {
        if (autoAlignSwerve == null) {
            autoAlignSwerve = new DefaultTeleopSub();
        }
        return autoAlignSwerve;
    }

    private DefaultTeleopSub() {


    }
}
