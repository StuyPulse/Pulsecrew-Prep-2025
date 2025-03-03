package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

public class Motors {

    public interface Swerve {
        public interface Turn {
            SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(true).smartCurrentLimit(200).openLoopRampRate(0.25).idleMode(IdleMode.kBrake);
            
            double kP = 4.5;
            double kI = 0.0;
            double kD = 0.05;
        }
        public interface Drive {
            double kS = 0.2825;
            double kV = 2.3716;
            double kA = 0.075654;

            double kP = 6.7279E-06;
            double kI = 0.0;
            double kD = 0.0;
    
            double MAX_MODULE_SPEED = 5.0;
            double MODULE_VELOCITY_DEADBAND = 0.02;
            Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKP(kP)
                .withKI(kI)
                .withKD(kD);
            
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
                .withTorqueClosedLoopRampPeriod(0.25);
            
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(65);

            FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(1/Encoder.POSITION_CONVERSION);

            TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(slot0Configs)
                .withMotorOutput(motorOutputConfigs)
                .withClosedLoopRamps(closedLoopRampsConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withFeedback(feedbackConfigs);
            
        }

        public interface Encoder {
            double WHEEL_DIAMETER = Units.inchesToMeters(4);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
            double GEAR_RATIO = 5.36;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        }
    }
}