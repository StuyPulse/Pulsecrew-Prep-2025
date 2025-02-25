package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Motors;

public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor;

    private SparkMax turnMotor;
    private CANcoder turnEncoder;

    private Translation2d translationOffset;
    private Rotation2d angleOffset;

    private SwerveModuleState targetState;

    private PIDController turnController;

    private SparkMaxConfig turnMotorConfig;

    private String id;

    public SwerveModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int driveMotorPort, int turnMotorPort, int turnEncoderPort){
        driveMotor = new TalonFX(driveMotorPort);
        
        turnMotor = new SparkMax(turnMotorPort, MotorType.kBrushless);
        turnEncoder = new CANcoder(turnEncoderPort);

        turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);

        Motors.Swerve.MOTOR_CONFIG.configure(driveMotor);
        turnMotorConfig = new SparkMaxConfig();

        turnMotorConfig.inverted(false);
        turnMotorConfig.idleMode(IdleMode.kBrake);
        turnMotorConfig.smartCurrentLimit(1, 80);

        turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        targetState = new SwerveModuleState();

        this.translationOffset = translationOffset;
        this.angleOffset = angleOffset;
        
        this.id = id;
    }

    public interface Turn {
        double kP = 6.0;
        double kI = 0.0;
        double kD = 0.15;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations());
    }

    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public String getId() {
        return id;
    }

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = state;
        targetState.optimize(getAngle());
    }

    @Override
    public void periodic() {
        driveMotor.setControl(new MotionMagicVelocityVoltage(getTargetState().speedMetersPerSecond));
        turnMotor.setVoltage(turnController.calculate(getAngle().getDegrees(), targetState.angle.getDegrees()));
    }
}
