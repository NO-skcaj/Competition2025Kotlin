package org.team3824.frc2025.subsystems;

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team3824.frc2025.LedConstants


object LEDs : SubsystemBase()
{
    var m_ledMode: LEDMode = LEDMode.Off;            // The LED mode

     var m_firstPixelHue: Int = 0;  // Store the hue of the first pixel for rainbow mode
     var m_cycleCounter:  Int = 0  // Counter for dynamic LED modes

     // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
     // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
    var m_scrollingRainbow: LEDPattern = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(1 / 120.0));

     // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
     // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
    var m_shooting: LEDPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlack)
         .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(1 / 120.0));

    var m_led: AddressableLED = AddressableLED(LedConstants.PwmPort);

    var m_ledBuffer: AddressableLEDBuffer = AddressableLEDBuffer(LedConstants.Length);  // Instatntiate the LED data buffer


    /// @brief Class to support an addressable LED string.
    init
    {
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(LedConstants.Length);

        // Set the default mode
        SetMode(LEDMode.Off);

        // Initialize the LED data
        m_led.setData(m_ledBuffer);

        // Start the addressable LED communications
        m_led.start();
    }

    /// @brief This method will be called once periodically.
    fun Periodic()
    {
        when (m_ledMode)
        {
            LEDMode.Off               -> print("LEDs set: Off")
            LEDMode.SolidGreen        -> print("LEDs set: SolidGreen")
            LEDMode.SolidRed          -> print("LEDs set: Solid Red")
            LEDMode.HvaColors         -> HvaColors()
            LEDMode.Strobe            -> Strobe()
            LEDMode.ShootingAnimation -> m_shooting.applyTo(m_ledBuffer)
            LEDMode.Rainbow           -> m_scrollingRainbow.applyTo(m_ledBuffer)
        }

        // Set the LEDs
        m_led.setData(m_ledBuffer)
    }

    /// @brief Setting the Led's mode to the given parameter.
    /// @param LEDMode mode to set the Leds.
    fun SetMode(ledMode: LEDMode)
    {
        // Remember the LED mode
        m_ledMode = ledMode

        // Set the LEDs based on the LED mode
        when (m_ledMode)
        {
            LEDMode.Off -> SolidColor(0, 0, 0)
            LEDMode.SolidGreen -> SolidColor(0, LedConstants.Green, 0)
            LEDMode.SolidRed -> SolidColor(LedConstants.Red, 0, 0)
            LEDMode.HvaColors -> {m_cycleCounter = 0; HvaColors();}
            LEDMode.Strobe -> {m_cycleCounter = 0; Strobe();}
            else -> print("LED set: unsure")
        }

        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    /// @brief Method to support setting the LED string to the specified solid color.
    /// @param red The red component of the LED color.
    /// @param green The green component of the LED color.
    /// @param blue The blue component of the LED color.
    fun SolidColor(red: Int, green: Int, blue: Int)
    {
        // Set the value for every pixel
        for (ledIndex in 0..LedConstants.Length) {
            m_ledBuffer.setRGB(ledIndex,red   / LedConstants.Brightness,
                                        green / LedConstants.Brightness,
                                        blue  / LedConstants.Brightness)
        }
    }

    /// @brief Method to support setting the LED string to HVA alternating color.
    fun HvaColors()
    {
        var firstColor:  Int = LedConstants.Blue;
        var secondColor: Int = 0;

        // Alternate the colors
        if (m_cycleCounter % LedConstants.HvaDelay < LedConstants.HvaDelay / 2)
        {
            firstColor  = 0;
            secondColor = LedConstants.Blue;
        }

        // For every pixel
        for (ledIndex in 0..LedConstants.Length)
        {
            // Set the color based on the pixel index
            if (ledIndex % 2 == 0)
                m_ledBuffer.setRGB(ledIndex,0, 0, firstColor / LedConstants.Brightness);
            else
                m_ledBuffer.setRGB(ledIndex,0, 0, secondColor / LedConstants.Brightness);
        }

        // Update the cycle counter
        m_cycleCounter++;
    }

    /// @brief Method to strobe the LED string.
    fun Strobe()
    {
        if (m_cycleCounter % LedConstants.StrobeDelay == 0)
            SolidColor(LedConstants.Red, LedConstants.Green, LedConstants.Blue);
        else
            SolidColor(0, 0, 0);

        // Update the cycle counter
        m_cycleCounter++;
    }
};
