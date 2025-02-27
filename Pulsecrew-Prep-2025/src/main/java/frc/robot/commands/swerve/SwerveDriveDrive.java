package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class SwerveDriveDrive extends Command {
    
    private XboxController driver;
    private SwerveDrive swerve;

    public SwerveDriveDrive(XboxController driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    public interface SwerveDriver {
        // Low Pass Filter and deadband for Driver Controls
        double DRIVE_DEADBAND = 0.05; //Driver Settings/Speed Deadband
        double TURN_DEADBAND = 0.05; //Driver Settings/Turn Deadband

        // Slew Rate Limiter Constants
        double DRIVE_POWER = 2;
        double DRIVE_SLEW_RATE = 8; //Driver Settings/Drive Slew Rate
        double DRIVE_RC = 0.2; //Driver Settings/Drive RC Constant

        double TURN_POWER = 1;
        double TURN_SLEW_RATE = 10; //Driver Settings/Turn Slew Rate
        double TURN_RC = 0.05; //Driver Settings/Drive RC Constant

        double MAX_TELEOP_SPEED = 5.55; //"Driver Settings/Drive/Max Speed"
        double MAX_TELEOP_TURNING = 6.0; //"Driver Settings/Turn/Max Turning"
    }

    @Override
    public void execute() {
        Translation2d speed = filterSpeed(new Translation2d(driver.getLeftX(), driver.getLeftY()));
        double rotation = filterTurn(driver.getRightX());

        swerve.drive(speed, rotation);
    }

    private final SlewRateLimiter speedSlew = new SlewRateLimiter(SwerveDriver.DRIVE_SLEW_RATE);
    private final LinearFilter speedFilter = LinearFilter.singlePoleIIR(SwerveDriver.DRIVE_RC, 0.02);
    
    public Translation2d filterSpeed(Translation2d speed) {
        if (speed.getNorm() < SwerveDriver.DRIVE_DEADBAND) {
            return new Translation2d();
        }
        else {
            double xSpeed = speed.getX();
            double ySpeed = speed.getY();

            xSpeed = Math.copySign(Math.pow(xSpeed, SwerveDriver.DRIVE_POWER), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, SwerveDriver.DRIVE_POWER), ySpeed);

            xSpeed = speedSlew.calculate(xSpeed);
            ySpeed = speedSlew.calculate(ySpeed);

            xSpeed = speedFilter.calculate(xSpeed);
            ySpeed = speedFilter.calculate(ySpeed);

            return new Translation2d(xSpeed, ySpeed).times(SwerveDriver.MAX_TELEOP_SPEED);
        }        
    }

    private final SlewRateLimiter turnSlew = new SlewRateLimiter(SwerveDriver.TURN_SLEW_RATE);
    private final LinearFilter turnFilter = LinearFilter.singlePoleIIR(SwerveDriver.TURN_RC, 0.02);
    public double filterTurn(double turn) {
        if (Math.abs(turn) < SwerveDriver.TURN_DEADBAND) {
            return 0;
        }
        else {
            turn = Math.copySign(Math.pow(turn, SwerveDriver.TURN_POWER), turn);
            turn = turnSlew.calculate(turn);
            turn = turnFilter.calculate(turn);

            return turn * SwerveDriver.MAX_TELEOP_TURNING;
        }
    }

}
