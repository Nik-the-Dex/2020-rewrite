package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;

public class ColorWheel extends SubsystemBase
{
    public  final SparkMax        WOFmotor   = new SparkMax(frc.robot.Constants.WOF_MOTOR, MotorType.kBrushless);
    public  final RelativeEncoder WOFencoder = WOFmotor.getEncoder();

    private final DoubleSolenoid wofCylinder = new DoubleSolenoid(
        frc.robot.Constants.PCM1,
        PneumaticsModuleType.CTREPCM,
        frc.robot.Constants.WOF_FWD,
        frc.robot.Constants.WOF_REV
    );

    private static final I2C.Port     i2cPort     = I2C.Port.kOnboard;
    private static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher  = new ColorMatch();
    private final Color kBlueTarget   = new Color(0.127, 0.424, 0.444);
    private final Color kGreenTarget  = new Color(0.166, 0.577, 0.257);
    private final Color kRedTarget    = new Color(0.520, 0.346, 0.133);
    private final Color kYellowTarget = new Color(0.310, 0.564, 0.125);
    private final Color kNothing      = new Color(0.291, 0.468, 0.239);

    public ColorWheel()
    {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kNothing);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        WOFmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        wofCylinder.set(Value.kForward);
    }

    public void toggleWOFCylinder()
    {
        if (wofCylinder.get() == Value.kForward)
        {
            wofCylinder.set(Value.kReverse);
        }
        else
        {
            wofCylinder.set(Value.kForward);
        }
    }

    public void rotationControl(int position)
    {
        if (WOFencoder.getPosition() < position)
        {
            WOFmotor.set(0.3);
        }
        else
        {
            WOFmotor.set(0);
            IO.mainController.setRumble(RumbleType.kLeftRumble, 1.0);
            IO.wofRunning = false;
        }
    }

    public void positionControl()
    {
        String           gameData      = DriverStation.getGameSpecificMessage();
        Color            detectedColor = colorSensor.getColor();
        ColorMatchResult match         = m_colorMatcher.matchClosestColor(detectedColor);

        if (gameData.isEmpty())
        {
            return;
        }

        switch (gameData.charAt(0))
        {
            case 'B':
                if (match.color != kRedTarget)
                {
                    WOFmotor.set(0.25);
                }
                else
                {
                    WOFmotor.set(0);
                    IO.mainController.setRumble(RumbleType.kLeftRumble, 1);
                    IO.wofRunning = false;
                }
                break;

            case 'G':
                if (match.color != kYellowTarget)
                {
                    WOFmotor.set(0.25);
                }
                else
                {
                    WOFmotor.set(0);
                    IO.mainController.setRumble(RumbleType.kLeftRumble, 1);
                    IO.wofRunning = false;
                }
                break;

            case 'R':
                if (match.color != kBlueTarget)
                {
                    WOFmotor.set(0.25);
                }
                else
                {
                    WOFmotor.set(0);
                    IO.mainController.setRumble(RumbleType.kLeftRumble, 1);
                    IO.wofRunning = false;
                }
                break;

            case 'Y':
                if (match.color != kGreenTarget)
                {
                    WOFmotor.set(0.25);
                }
                else
                {
                    WOFmotor.set(0);
                    IO.mainController.setRumble(RumbleType.kLeftRumble, 1);
                    IO.wofRunning = false;
                }
                break;

            default:
                break;
        }
    }

    public void runWOF()
    {
        if (DriverStation.getGameSpecificMessage().isEmpty())
        {
            rotationControl(300);
        }
        else
        {
            positionControl();
        }
    }

    @Override
    public void periodic()
    {
        colorSensor.getColor();
    }
}
