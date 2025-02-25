package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class TankDriveForwards extends Command {

    private double distance;
    private double speed;

    private TankDrive tank;

    public TankDriveForwards(double distance, double speed) {
        this.distance = distance;
        this.speed = speed;

        tank = TankDrive.getInstance();

        addRequirements();
    }

    @Override
    public void execute() {
        tank.setSpeed(speed);
    }

    public void end() {
        tank.setSpeed(0);
    }

    public boolean isFinished() {
        return tank.getDistance() >= distance;
    }

    
}
