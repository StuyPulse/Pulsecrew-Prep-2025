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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrive extends SubsystemBase{

    private final static SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            // new SwerveModule("Front Left", new Translation2d(0, 0), Rotation2d.fromDegrees(0), 15, 14, 5),
            // new SwerveModule("Front Right", new Translation2d(0, 0), Rotation2d.fromDegrees(0), 13, 12, 4),
            // new SwerveModule("Back Left", new Translation2d(0, 0), Rotation2d.fromDegrees(0), 0, 1, 3),
            // new SwerveModule("Back Right", new Translation2d(0, 0), Rotation2d.fromDegrees(0), 2, 3, 2) 
            new SwerveModule("Front Left", new Translation2d(0, 0), Rotation2d.fromDegrees(-53.701172), 16, 15, 4),
            new SwerveModule("Front Right", new Translation2d(0, 0), Rotation2d.fromDegrees(21.796875), 10, 17, 1),
            new SwerveModule("Back Left", new Translation2d(0, 0), Rotation2d.fromDegrees(96.591797), 14, 13, 3),
            new SwerveModule("Back Right", new Translation2d(0, 0), Rotation2d.fromDegrees(-141.152344), 12, 11, 2) 
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
        resetOdometry(new Pose2d());
    }

    // Constants    
    public interface Swerve {
        //TODO: Robot Specific
        double WIDTH = Units.inchesToMeters(21);
        double LENGTH = Units.inchesToMeters(21);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(18);

        double MODULE_VELOCITY_DEADBAND = 0.05; //m/s

        //XXX: Motor Specfic
        double MAX_MODULE_SPEED = 5.88;
        double MAX_MODULE_TURN = 6.28; 
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];

        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getTranslationOffset();
        }

        return offsets;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }

        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }

        return states;
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                String.format("State count mismatch error: %d states does not equal %d modules", states.length, modules.length)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(Translation2d vel, double rot) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel.getX(), vel.getY(), rot, getPose().getRotation());

        Pose2d robotVel = new Pose2d(
            0.02 * speeds.vxMetersPerSecond,
            0.02 * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(0.02 * speeds.omegaRadiansPerSecond)
        );

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
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Field2d getField() {
        return field;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void updateOdometry() {
        odometry.update(getGyroAngle(), getModulePositions());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void initModule2ds(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            modules2ds[i] = field.getObject(modules[i].getId() + "-2d");
        }
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
    }
}
