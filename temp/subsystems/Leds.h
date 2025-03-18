#pragma once

#include <array>

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/LEDPattern.h>

#include "Constants.h"

/// @brief modes for the LED string.
enum LedMode
{
    Off,
    SolidGreen,
    SolidRed,
    HvaColors,
    Strobe,
    ShootingAnimation,
    Rainbow
};

class Leds : public frc2::SubsystemBase
{
    public:
    
        /// @brief Class to support an addressable LED string.
        Leds()
        {
            // Length is expensive to set, so only set it once, then just update data
            m_led.SetLength(LedConstants::Length);
        
            // Set the default mode
            SetMode(LedMode::Off);
        
            // Intialize the LED data
            m_led.SetData(m_ledBuffer);
        
            // Start the addressable LED communications
            m_led.Start();
        }
        
        /// @brief This method will be called once periodically.
        void Periodic()
        {
            switch (m_ledMode)
            {
                case LedMode::Off:
                case LedMode::SolidGreen:
                case LedMode::SolidRed:
                    break;
        
                case LedMode::HvaColors:
                    HvaColors();
                    break;
        
                case LedMode::Strobe:
                    Strobe();
                    break;
        
                case LedMode::ShootingAnimation:
                {
                    // Apply the shootime pattern to the data buffer
                    m_shooting.ApplyTo(m_ledBuffer);
                    break;
                }
        
                case LedMode::Rainbow:
                {
                    // Run the rainbow pattern and apply it to the buffer
                    m_scrollingRainbow.ApplyTo(m_ledBuffer);
                    break;
                }
            }
        
            // Set the LEDs
            m_led.SetData(m_ledBuffer);
        }
        
        /// @brief Setting the Led's mode to the given parameter.
        /// @param ledMode mode to set the Leds.
        void SetMode(LedMode ledMode)
        {
            // Remember the LED mode
            m_ledMode = ledMode;
        
            // Set the LEDs based on the LED mode
            switch (m_ledMode)
            {
            case LedMode::Off:
                SolidColor(0, 0, 0);
                break;
        
            case LedMode::SolidGreen:
                SolidColor(0, LedConstants::Green, 0);
                break;
        
            case LedMode::SolidRed:
                SolidColor(LedConstants::Red, 0, 0);
                break;
        
            case LedMode::HvaColors:
                m_cycleCounter = 0;
                HvaColors();
                break;
        
            case LedMode::Strobe:
                m_cycleCounter = 0;
                Strobe();
                break;
        
            default:
                break;
            }
        
            // Set the LEDs
            m_led.SetData(m_ledBuffer);
        }
        
        /// @brief Method to support setting the LED string to the specified solid color.
        /// @param red The red component of the LED color.
        /// @param green The green component of the LED color.
        /// @param blue The blue component of the LED color.
        void SolidColor(int red, int green, int blue)
        {
            // Set the value for every pixel
            for (auto ledIndex = 0; ledIndex < LedConstants::Length; ledIndex++)
                m_ledBuffer[ledIndex].SetRGB(red * LedConstants::Brightness, green * LedConstants::Brightness, blue * LedConstants::Brightness);
        }
        
        /// @brief Method to support setting the LED string to HVA alternating color.
        void HvaColors()
        {
            int firstColor  = LedConstants::Blue;
            int secondColor = 0;
        
            // Alternate the colors
            if (m_cycleCounter % LedConstants::HvaDelay < LedConstants::HvaDelay / 2)
            {
                firstColor  = 0;
                secondColor = LedConstants::Blue;
            }
        
            // For every pixel
            for (auto ledIndex = 0; ledIndex < LedConstants::Length; ledIndex++)
            {
                // Set the color based on the pixel index
                if (ledIndex % 2 == 0)
                    m_ledBuffer[ledIndex].SetRGB(0, 0, firstColor * LedConstants::Brightness);
                else
                    m_ledBuffer[ledIndex].SetRGB(0, 0, secondColor * LedConstants::Brightness);
            }
        
            // Update the cycle counter
            m_cycleCounter++;
        }
        
        /// @brief Method to strobe the LED string.
        void Strobe()
        {
            if (m_cycleCounter % LedConstants::StrobeDelay == 0)
                SolidColor(LedConstants::Red, LedConstants::Green, LedConstants::Blue);
            else
                SolidColor(0, 0, 0);
        
            // Update the cycle counter
            m_cycleCounter++;
        }
        
    

        LedMode m_ledMode;            // The LED mode

        int     m_firstPixelHue = 0;  // Store the hue of the first pixel for rainbow mode
        int     m_cycleCounter  = 0;  // Counter for dynamic LED modes

        // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
        // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
        frc::LEDPattern     m_scrollingRainbow = frc::LEDPattern::Rainbow(255, 128).ScrollAtAbsoluteSpeed(0.1_mps, units::meter_t{1 / 120.0});

        // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
        // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
        frc::LEDPattern     m_shooting = frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, std::array<frc::Color, 2>{frc::Color::kRed, frc::Color::kBlack}).
                                                          ScrollAtAbsoluteSpeed(0.5_mps, units::meter_t{1 / 120.0});

        frc::AddressableLED m_led{LedConstants::PwmPort};

        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledBuffer;  // Instatntiate the LED data buffer
};
