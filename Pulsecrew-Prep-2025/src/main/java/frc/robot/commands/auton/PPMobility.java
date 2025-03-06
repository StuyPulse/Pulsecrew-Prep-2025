package frc.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class PPMobility extends SequentialCommandGroup{
    public PPMobility(PathPlannerPath... paths) {
        addCommands(
            SwerveDrive.getInstance().followPathCommand(paths[0])
        );
    }
}
