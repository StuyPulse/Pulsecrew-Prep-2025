package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class SwerveDriveDrive extends Command{
    private SwerveDrive swerve;
    private XboxController driver;

    private final Supplier<Translation2d> speed;
    private final DoubleSupplier turn;
    // private final LinearFilter speedFilter;
    // private final LinearFilter turnFilter;

    public SwerveDriveDrive(XboxController driver) {
        swerve = SwerveDrive.getInstance();
        this.driver = driver;

        // speedFilter = LinearFilter.singlePoleIIR(SwerveDriver.DRIVE_RC, 0.02);
        // turnFilter = LinearFilter.singlePoleIIR(SwerveDriver.TURN_RC, 0.02);

        speed = () -> {
            Translation2d inputVel = getInputVelocity();
            if (inputVel.getNorm() < 0.07){
                return new Translation2d();
            }
            double x = inputVel.getX();
            double y = inputVel.getY();

            x /= inputVel.getNorm();
            y /= inputVel.getNorm();

            x *= x * 1.5;
            y *= y * 1.5; 

            // x = speedFilter.calculate(x);
            // y = speedFilter.calculate(y);
            
            return new Translation2d(x, y);
        };
        
        turn = () -> {
            double inputTurn = -driver.getRightX();

            if (Math.abs(inputTurn) < 0.07){
                return 0;
            }
            
            inputTurn *= Math.signum(inputTurn) * inputTurn;

            inputTurn *= 2 * Math.PI;
            // inputTurn = turnFilter.calculate(inputTurn);

            return inputTurn;
        };
        
        addRequirements(swerve);
    }

    private Translation2d getInputVelocity() {
        return new Translation2d(driver.getLeftY(), -driver.getLeftX());
    }

    @Override
    public void execute() {
        swerve.drive(speed.get(), turn.getAsDouble());
    }
}
