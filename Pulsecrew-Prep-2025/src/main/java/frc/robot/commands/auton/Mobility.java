package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.swerve.SwerveDriveForward;

public class Mobility extends SequentialCommandGroup {
    
    public Mobility() {
        int speed = -1;
        if (Robot.isBlue()) {
            speed = -speed;
        }
        addCommands(
            new WaitCommand(1),
            new SwerveDriveForward(speed).withTimeout(1)
        );
    }

}
