package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Motors;
import frc.robot.constants.Motors.TalonFXConfig;

public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor;

    private SparkMax turnMotor;
    private CANcoder turnEncoder;

    private Translation2d translationOffset;
    private Rotation2d angleOffset;

    private SwerveModuleState targetState;

    private PIDController turnController;

    private String id;

    public SwerveModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int driveMotorPort, boolean driveInverted, int turnMotorPort, int turnEncoderPort){
        driveMotor = new TalonFX(driveMotorPort, "Swerve Drive Drive");
        
        turnMotor = new SparkMax(turnMotorPort, MotorType.kBrushless);
        turnEncoder = new CANcoder(turnEncoderPort, "Swerve Drive Drive");

        turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);
        turnController.enableContinuousInput(-180, 180);

        
        if (driveInverted) {
            Motors.Swerve.MOTOR_CONFIG.withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .configure(driveMotor);
            // TalonFXConfig driveMotorConfig = new TalonFXConfig();
            // driveMotorConfig.withInvertedValue(InvertedValue.CounterClockwise_Positive);
            // driveMotorConfig.configure(driveMotor);
        } else {
            Motors.Swerve.MOTOR_CONFIG.withInvertedValue(InvertedValue.Clockwise_Positive)
            .configure(driveMotor);
        }

        turnMotor.configure(Motors.Swerve.TURN_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        targetState = new SwerveModuleState();

        this.translationOffset = translationOffset;
        this.angleOffset = angleOffset;
        
        this.id = id;
    }

    public interface Turn {
        double kP = 0.1;
        double kI = 0.0;
        double kD = 0.0;
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
        state.optimize(getAngle());
        state.cosineScale(getAngle());
        targetState = state;
    }

    @Override
    public void periodic() {
        
        // if (targetState.speedMetersPerSecond < Motors.Swerve.Drive.MODULE_VELOCITY_DEADBAND) {
        //     driveMotor.setVoltage(0);
        //     turnMotor.setVoltage(0);
        // } 
        // else {
            driveMotor.setControl(new MotionMagicVelocityVoltage(getTargetState().speedMetersPerSecond));
            double turnVoltage = turnController.calculate(getAngle().getDegrees(), targetState.angle.getDegrees());
            turnMotor.setVoltage(turnVoltage);
        // } 

        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Setpoint", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Setpoint", turnController.getSetpoint());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Current", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", turnVoltage);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", turnController.getError());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Angle", targetState.angle.getDegrees());
        
    }
}
