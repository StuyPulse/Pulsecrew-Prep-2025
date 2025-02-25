package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.tank.TankDriveForwards;

public class Mobility extends SequentialCommandGroup{
    public Mobility() {
        addCommands(
            new WaitCommand(4),
            new TankDriveForwards(1, 1)
        );
    }
}
