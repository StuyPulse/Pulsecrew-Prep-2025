package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.constants.Motors;
import frc.robot.subsystems.Odometry.Odometry;
import frc.robot.constants.Constants.Swerve.*;
import frc.robot.constants.Ports;

public class SwerveDrive extends SubsystemBase{

    private final static SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            new SwerveModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER),
            new SwerveModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER),
            new SwerveModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER),
            new SwerveModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER)
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    // private final AHRS gyro;
    private final Pigeon2 gyro;
    // private final SwerveDriveOdometry odometry;
    // private final Field2d field;
    private final FieldObject2d[] modules2ds;

    protected SwerveDrive(SwerveModuleImpl... modules) {
        this.modules = modules;

        gyro = new Pigeon2(9, "Swerve Drive Drive");

        kinematics = new SwerveDriveKinematics(getModuleOffsets());

        modules2ds = new FieldObject2d[modules.length];
    }

    
    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            modules2ds[i] = field.getObject(modules[i].getName()+"-2d");
            modules2ds[i].setPose(Robot.isBlue() ? modules2ds[i].getPose() : Field.transformToOppositeAlliance(modules2ds[i].getPose()));
        }
    }

    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];

        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getModuleOffset();
        }

        return locations;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModule getModule(String id) {
        for (SwerveModule module : modules)
            if (module.getName().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    /** MODULE STATES API **/
    public void drive(Translation2d velocity, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(),
                velocity.getY(),
                omega,
                Odometry.getInstance().getRotation());

        Pose2d robotVel = new Pose2d(
            0.02 * speeds.vxMetersPerSecond,
            0.02 * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(0.02 * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / 0.02,
            twistVel.dy / 0.02,
            twistVel.dtheta / 0.02
        ));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        double x = robotSpeed.vxMetersPerSecond;
        double y = robotSpeed.vyMetersPerSecond;
        double magnitude = Math.sqrt((Math.pow(x,2) + Math.pow(y,2)));
        if (magnitude >= 3.0) {
            x = x * 3.0 / magnitude;
            y = y * 3.0 / magnitude;
        }
        Translation2d xy = new Translation2d(x, y);
        
        robotSpeed.vxMetersPerSecond = xy.getX();
        robotSpeed.vyMetersPerSecond = xy.getY();

        if (robotSpeed.omegaRadiansPerSecond > 360.0) {
            robotSpeed.omegaRadiansPerSecond = 360.0;
        }

        if (robotSpeed.omegaRadiansPerSecond < -360.0) {
            robotSpeed.omegaRadiansPerSecond = -360.0;
        }
        
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    private static SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND)
            return state;

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Motors.Swerve.Drive.MAX_MODULE_SPEED);

        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(filterModuleState(states[i]));
        }
    }

    // /** PATH FOLLOWING **/
    // public Command followPathCommand(PathPlannerPath path) {
    //     return AutoBuilder.followPath(path);
    // }

    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public double getGyroYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getGyroPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getGyroRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    /** KINEMATICS **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        SwerveModuleState[] state = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
        setModuleStates(state);
    }

    // public void configureAutoBuilder() {        
    //     try{
    //         Odometry odometry = Odometry.getInstance();

    //         AutoBuilder.configure(
    //             odometry::getPose,
    //             odometry::reset,
    //             this::getChassisSpeeds,
    //             (speeds, feedforwards) -> setChassisSpeeds(speeds),
    //             new PPHolonomicDriveController(
    //                 Settings.Swerve.Alignment.XY,
    //                 Settings.Swerve.Alignment.THETA
    //             ),
    //             RobotConfig.fromGUISettings(),
    //             () -> false,
    //             instance
    //         );
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //     }
    // }

    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for (int i = 0; i < modules.length; ++i) {
            Pose2d modulePose = new Pose2d(
                pose.getTranslation().plus(modules[i].getModuleOffset().rotateBy(angle)),
                modules[i].getState().angle.plus(angle)
            );
            modules2ds[i].setPose(Robot.isBlue() ? modulePose : Field.transformToOppositeAlliance(modulePose));
        }

        SmartDashboard.putNumber("Swerve/Gyro Angle (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Pitch (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Roll (deg)", getGyroRoll());

        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getAccelerationX().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getAccelerationY().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getAccelerationZ().getValueAsDouble());
    }


    // public SwerveDrive(SwerveModule... modules) {
    //     this.modules = modules;
    //     kinematics = new SwerveDriveKinematics(getModuleOffsets());
    //     // gyro = new AHRS(NavXComType.kMXP_SPI);
    //     gyro = new Pigeon2(9, "Swerve Drive Drive");
    //     odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());
    //     field = new Field2d();
    //     modules2ds = new FieldObject2d[modules.length];
        
    //     initmodules2ds(field);
    //     reset(new Pose2d());
    // }

    // Constants    
    public interface Swerve {
        //TODO: Robot Specific
        double WIDTH = Units.inchesToMeters(18.75);
        double LENGTH = Units.inchesToMeters(18.75);

        double MODULE_VELOCITY_DEADBAND = 0.05; //m/s
    }

    // public void initmodules2ds(Field2d field) {
    //     for(int i=0; i<modules.length; i++) {
    //         modules2ds[i] = field.getObject(modules[i].getId() + "-2d");
    //     }
    // }

    // public Translation2d[] getModuleOffsets() {
    //     Translation2d[] offsets = new Translation2d[modules.length];

    //     for(int i=0; i<modules.length; i++) {
    //         offsets[i] = modules[i].getTranslationOffset();
    //     }

    //     return offsets;
    // }

    // public SwerveModuleState[] getModuleStates() {
    //     SwerveModuleState[] states = new SwerveModuleState[modules.length];
        
    //     for(int i=0; i<modules.length; i++) {
    //         states[i] = modules[i].getModuleState();
    //     }

    //     return states;
    // }

    // public SwerveModulePosition[] getModulePositions() {
    //     SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

    //     for(int i=0; i<modules.length; i++) {
    //         positions[i] = modules[i].getModulePosition();
    //     }

    //     return positions;
    // }

    // public ChassisSpeeds getChassisSpeeds() {
    //     return kinematics.toChassisSpeeds(getModuleStates());
    // }

    // public SwerveModuleState filterModuleState(SwerveModuleState state) {
    //     if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND) {
    //         return state;
    //     }

    //     return new SwerveModuleState(0, state.angle);
    // }

    // public void setModuleStates(SwerveModuleState... states) {
    //     if (states.length != modules.length) {
    //         throw new IllegalArgumentException(
    //             String.format("State count mismatch error: %d states does not equal %d modules", states.length, modules.length)
    //         );
    //     }

    //     SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

    //     for (int i = 0; i < modules.length; i++) {
    //         modules[i].setTargetState(filterModuleState(states[i]));
    //     }
    // }

    // public void setChassisSpeeds(ChassisSpeeds speeds) {
    //     SmartDashboard.putNumber("Swerve/Chassis Target X", speeds.vxMetersPerSecond);
    //     SmartDashboard.putNumber("Swerve/Chassis Target Y", speeds.vyMetersPerSecond);
    //     SmartDashboard.putNumber("Swerve/Chassis Target Omega", speeds.omegaRadiansPerSecond);

    //     setModuleStates(kinematics.toSwerveModuleStates(speeds));
    // }

    // public void drive(Translation2d velocity, double rotation) {
    //     ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), -velocity.getY(), -rotation, getPose().getRotation());
        
    //     Pose2d robotVel = new Pose2d(
    //         0.02 * speeds.vxMetersPerSecond,
    //         0.02 * speeds.vyMetersPerSecond,
    //         Rotation2d.fromRadians(0.02 * speeds.omegaRadiansPerSecond));

    //     Twist2d twistVel = new Pose2d().log(robotVel);
        
    //     setChassisSpeeds(new ChassisSpeeds(
    //         twistVel.dx / 0.02,
    //         twistVel.dy / 0.02,
    //         twistVel.dtheta / 0.02
    //     ));
    // }
    
    // public void stop() {
    //     setChassisSpeeds(new ChassisSpeeds());
    // }

    // public Rotation2d getGyroAngle() {
    //     return gyro.getRotation2d();
    // }

    // public double getGyroYaw() {
    //     return gyro.getYaw().getValueAsDouble();
    // }

    // public double getGyroPitch() {
    //     return gyro.getPitch().getValueAsDouble();
    // }

    // public double getGyroRoll() {
    //     return gyro.getRoll().getValueAsDouble();
    // }
    
    // public double getForwardAccelerationGs() {
    //     return gyro.getAccelerationY().getValueAsDouble();
    // }

    // public SwerveDriveKinematics getKinematics() {
    //     return kinematics;
    // }

    // public void setXMode() {
    //     setModuleStates(
    //             new SwerveModuleState[] {
    //                 new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
    //                 new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
    //                 new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    //                 new SwerveModuleState(0, Rotation2d.fromDegrees(135))
    //             }
    //         );
    // }

    // public Field2d getField() {
    //     return field;
    // }

    // public Pose2d getPose() {
    //     return odometry.getPoseMeters();
    // }

    // public void reset(Pose2d pose) {
    //     odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    // }

    // public void updateOdometry() {
    //     odometry.update(getGyroAngle(), getModulePositions());
    // }

    // @Override
    // public void periodic() {
    //     updateOdometry();
    //     Pose2d pose = getPose();

    //     for (int i = 0; i < modules.length; i++) {
    //         modules2ds[i].setPose(new Pose2d(
    //             pose.getTranslation().plus(modules[i].getTranslationOffset().rotateBy(pose.getRotation())),
    //             modules[i].getAngle().plus(pose.getRotation())
    //         ));
    //     }

    //     SmartDashboard.putNumber("Swerve/Gyro/Angle (deg)", getGyroAngle().getDegrees());
    //     SmartDashboard.putNumber("Swerve/Gyro/Pitch (deg)", getGyroPitch());
    //     SmartDashboard.putNumber("Swerve/Gyro/Roll (deg)", getGyroRoll());

    //     SmartDashboard.putNumber("Swerve/Forward Acceleration  (Gs)", getForwardAccelerationGs());
    //     SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getAccelerationX().getValueAsDouble());
    //     SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getAccelerationY().getValueAsDouble());
    //     SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getAccelerationZ().getValueAsDouble());

    //     SmartDashboard.putNumber("Swerve/Chassis X Speed", getChassisSpeeds().vxMetersPerSecond);
    //     SmartDashboard.putNumber("Swerve/Chassis Y Speed", getChassisSpeeds().vyMetersPerSecond);
    //     SmartDashboard.putNumber("Swerve/Chassis Rotation", getChassisSpeeds().omegaRadiansPerSecond);
    // }
}
