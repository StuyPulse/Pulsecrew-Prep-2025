package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase{

    private final static SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            // new SwerveModule("Front Left", new Translation2d(Swerve.LENGTH * 0.0, Swerve.WIDTH * 0.0), Rotation2d.fromDegrees(125.507812), 16, false, 15, 4),
            // new SwerveModule("Front Right", new Translation2d(Swerve.LENGTH * 0, Swerve.WIDTH * -0), Rotation2d.fromDegrees(-158.378906), 10, true, 17, 1),
            // new SwerveModule("Back Left", new Translation2d(Swerve.LENGTH * -0, Swerve.WIDTH * 0), Rotation2d.fromDegrees(-84.287109), 14, false, 13, 3),
            // new SwerveModule("Back Right", new Translation2d(Swerve.LENGTH * -0, Swerve.WIDTH * -0), Rotation2d.fromDegrees(37.353516), 12, true, 11, 2) 
            new SwerveModule("Front Left", new Translation2d(Swerve.LENGTH * 0.5, Swerve.WIDTH * -0.5), Rotation2d.fromDegrees(125.507812), 16, false, 17, 2),
            new SwerveModule("Front Right", new Translation2d(Swerve.LENGTH * 0.5, Swerve.WIDTH * -0.5), Rotation2d.fromDegrees(-158.378906), 14, true, 15, 4),
            new SwerveModule("Back Left", new Translation2d(Swerve.LENGTH * -0.5, Swerve.WIDTH * 0.5), Rotation2d.fromDegrees(-84.287109), 10, false, 11, 3),
            new SwerveModule("Back Right", new Translation2d(Swerve.LENGTH * -0.5, Swerve.WIDTH * 0.5), Rotation2d.fromDegrees(37.353516), 12, true, 13, 1) 
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;
    private final FieldObject2d[] modules2ds;

    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(NavXComType.kMXP_SPI);
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());
        field = new Field2d();
        modules2ds = new FieldObject2d[modules.length];
        
        initModule2ds(field);
        reset(new Pose2d());
    }

    // Constants    
    public interface Swerve {
        //TODO: Robot Specific
        double WIDTH = Units.inchesToMeters(18.75);
        double LENGTH = Units.inchesToMeters(18.75);

        double MODULE_VELOCITY_DEADBAND = 0.05; //m/s

        //XXX: Motor Specfic
        double MAX_MODULE_SPEED = 5.88;
        double MAX_MODULE_TURN = 6.28; 
    }

    public void initModule2ds(Field2d field) {
        for(int i=0; i<modules.length; i++) {
            modules2ds[i] = field.getObject(modules[i].getId() + "-2d");
        }
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];

        for(int i=0; i<modules.length; i++) {
            offsets[i] = modules[i].getTranslationOffset();
        }

        return offsets;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        
        for(int i=0; i<modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for(int i=0; i<modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND) {
            return state;
        }

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                String.format("State count mismatch error: %d states does not equal %d modules", states.length, modules.length)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(filterModuleState(states[i]));
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("Swerve/Chassis Target X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Omega", speeds.omegaRadiansPerSecond);

        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(Translation2d velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), -velocity.getY(), -rotation, getPose().getRotation());
        
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
    
    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroYaw() {
        return getGyroAngle();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    
    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        setModuleStates(
                new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(135))
                }
            );
    }

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void updateOdometry() {
        odometry.update(getGyroAngle(), getModulePositions());
    }

    @Override
    public void periodic() {
        updateOdometry();
        Pose2d pose = getPose();

        for (int i = 0; i < modules.length; i++) {
            modules2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getTranslationOffset().rotateBy(pose.getRotation())),
                modules[i].getAngle().plus(pose.getRotation())
            ));
        }

        SmartDashboard.putNumber("Swerve/Gyro/Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Pitch (deg)", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Roll (deg)", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Forward Acceleration  (Gs)", getForwardAccelerationGs());
        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());

        SmartDashboard.putNumber("Swerve/Chassis X Speed", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Y Speed", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Rotation", getChassisSpeeds().omegaRadiansPerSecond);
    }
}
