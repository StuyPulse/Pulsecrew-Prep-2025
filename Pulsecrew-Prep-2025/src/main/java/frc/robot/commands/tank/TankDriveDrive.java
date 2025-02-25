package frc.robot.commands.tank;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class TankDriveDrive extends Command {

    private TankDrive tank;
    private XboxController driver;

    private double leftSpeed, rightSpeed;

    public TankDriveDrive(TankDrive tank, XboxController xbox) {
        this.tank = tank;
        this.driver = xbox;

        leftSpeed = driver.getLeftY();
        rightSpeed = driver.getRightY();

        addRequirements(tank);
    }

    public interface TankDriver {
        double DRIVE_DEADBAND = 0.5;

        double MAX_TELEOP_SPEED = 5.5;

    }

    @Override
    public void execute() {
        tank.tankDrive(leftSpeed, rightSpeed);
    }

    public boolean isFinished() {
        return false;
    }

}
