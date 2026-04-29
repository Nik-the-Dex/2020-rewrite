package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallMech extends SubsystemBase
{
    public  TalonFX  shooterMotor1   = new TalonFX(7);
    private TalonFX  shooterMotor2   = new TalonFX(8);
    public  SparkMax intakeMotor     = new SparkMax(frc.robot.Constants.BALLMECH_INTAKE, MotorType.kBrushless);
    private SparkMax intakeMotor2    = new SparkMax(6, MotorType.kBrushless);
    private SparkMax intakeMotor3    = new SparkMax(7, MotorType.kBrushless);
    public  SparkMax rollerMotor     = new SparkMax(frc.robot.Constants.BALLMECH_ROLLER, MotorType.kBrushed);

    private final DoubleSolenoid intakeCylinder = new DoubleSolenoid(
        frc.robot.Constants.PCM2,
        PneumaticsModuleType.CTREPCM,
        frc.robot.Constants.INTAKE_EXTENDER_FWD,
        frc.robot.Constants.INTAKE_EXTENDER_REV
    );

    private final Ultrasonic ballSensorIntake  = new Ultrasonic(4, 3);
   // private final Ultrasonic ballSensorShooter = new Ultrasonic(2, 3);

    private final VelocityVoltage velocityRequest      = new VelocityVoltage(0);
    private final DutyCycleOut    dutyCycleRequest      = new DutyCycleOut(0);
   final Follower shooterFollowerRequest = new Follower(shooterMotor1.getDeviceID(), null);

    public int     counter      = 0;
    public int     shooterState = 0;
    public int     rollerState  = 0;
    public int     intakeCounter = 0;

    private int ballCountState = 0;

    @SuppressWarnings("removal")
    public BallMech()
    {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kV = frc.robot.Constants.kV;
        shooterConfig.Slot0.kP = frc.robot.Constants.kP;
        shooterConfig.Slot0.kI = frc.robot.Constants.kI;
        shooterConfig.Slot0.kD = frc.robot.Constants.kD;
        shooterMotor1.getConfigurator().apply(shooterConfig);

        shooterMotor2.setControl(shooterFollowerRequest);

        Ultrasonic.setAutomaticMode(true);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig intake3Config = new SparkMaxConfig();
        intake3Config.follow(intakeMotor2, true);
        intakeMotor3.configure(intake3Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void intake(double speed)
    {
        intakeCounter++;
        extendIntake(true);
        if (intakeCounter > 20)
        {
            intakeMotor.set(speed);
            intakeMotor2.set(0.2);
        }
    }

    public void intakeTwo(double speed)
    {
        extendIntake(true);
        intakeMotor.set(speed);
        intakeMotor2.set(0.2);
    }

    public void stopIntake()
    {
        intakeMotor.set(0);
        intakeMotor2.set(0);
        extendIntake(false);
        intakeCounter = 0;
    }

    public void runRollerWithIntake()
    {
        if (ballSensorIntake.getRangeInches() < 4)
        {
            rollerMotor.set(0);
        }
        else
        {
            rollerMotor.set(-0.4);
        }
    }

    public void runRoller(double speed)
    {
        rollerMotor.set(speed);
    }

    public boolean ballIsInRobot()
    {
        return ballSensorIntake.getRangeInches() < 4;
    }

    public boolean setShooterMotors(double speed)
    {
        shooterMotor1.setControl(velocityRequest.withVelocity(speed));
        if (Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - speed) < 4.88)
        {
            counter++;
        }
        return counter > 30;
    }

    public void testShooterMotors(double speed)
    {
        shooterMotor1.setControl(velocityRequest.withVelocity(speed));
    }

    public void stopShooter()
    {
        shooterMotor1.setControl(dutyCycleRequest.withOutput(0));
        intakeMotor2.set(0);
    }

    public void variableDistanceShooter()
    {
        switch (shooterState)
        {
            case 0:
                counter = 0;
                if (setShooterMotors(-85.9))
                {
                    shooterState = 1;
                }
                break;

            case 1:
                intakeMotor2.set(-0.6);
                rollerMotor.set(-0.5);
                break;

            case 2:
                break;

            default:
                break;
        }
    }

    public void extendIntake(boolean extend)
    {
        if (extend)
        {
            intakeCylinder.set(Value.kReverse);
        }
        else
        {
            intakeCylinder.set(Value.kForward);
        }
    }

  /*   public void ballIndexing()
    {
        if (getBallCount() >= 3 && ballSensorShooter.getRangeInches() > 30)
        {
            runRoller(0.5);
        }
        else
        {
            runRoller(0);
        }
    } */

    private int getBallCount()
    {
        int count = 0;
        switch (ballCountState)
        {
            case 0:
                if (ballSensorIntake.getRangeInches() < 2)
                {
                    count++;
                    ballCountState = 1;
                }
                break;

            case 1:
                if (ballSensorIntake.getRangeInches() > 5)
                {
                    ballCountState = 0;
                }
                break;

            default:
                break;
        }
        return count;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("shooterVelocity",      shooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ballSensorIntake_in",  ballSensorIntake.getRangeInches());
      //  SmartDashboard.putNumber("ballSensorShooter_in", ballSensorShooter.getRangeInches());
    }
}
