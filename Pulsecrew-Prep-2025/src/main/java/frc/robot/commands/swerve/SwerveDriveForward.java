package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class SwerveDriveForward extends Command {
    
    private SwerveDrive swerve;
    private double speed;

    public SwerveDriveForward(double speed) {
        swerve = SwerveDrive.getInstance();
        this.speed = speed;
        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(speed, 0), 0);
    }
}
