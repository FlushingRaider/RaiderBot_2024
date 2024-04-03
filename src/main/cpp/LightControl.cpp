#include <array>

#include <frc/AddressableLED.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Const.hpp"

int firstPixelHue = 0;

bool topHit = false;

double timer = 0;

int color = 0;

std::array<frc::AddressableLED::LEDData, C_LedLength> Va_LED_outputBuffer;

void LightControl(
    TsENC_LightPatterns L_pattern)
{

    switch (L_pattern)
    {
    case E_LED_RAINBOW:
        // For every pixel
        for (int i = 0; i < C_LedLength; i++)
        {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            int pixelHue = (firstPixelHue + (i * 180 / C_LedLength)) % 180;
            // Set the value
            Va_LED_outputBuffer[i].SetHSV(pixelHue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;
        break;
    case E_LED_SOLIDWHITE:
        for (int i = 0; i < C_LedLength; i++)
        {
            Va_LED_outputBuffer[i].SetRGB(128, 128, 128);
        }
        break;
    case E_LED_FADEBLUE:

        for (int i = 0; i < C_LedLength; i++)
        {

            Va_LED_outputBuffer[i].SetRGB(0, 0, color);
        }

        if (color >= 255)
        {
            topHit = true;
        }
        if (color <= 0)
        {
            topHit = false;
        }

        if (topHit)
        {
            color -= 5;
        }
        else
        {
            color += 5;
        }

        break;

        case E_LED_FADERED:

        for (int i = 0; i < C_LedLength; i++)
        {

            Va_LED_outputBuffer[i].SetRGB(color, 0, 0);
        }

        if (color >= 255)
        {
            topHit = true;
        }
        if (color <= 0)
        {
            topHit = false;
        }

        if (topHit)
        {
            color -= 5;
        }
        else
        {
            color += 5;
        }

        break;

    /*TODO - 
        - green strobe
        - orange strobe
        - red strobe
        map to: belly intake full, amp intake full, roller up to speed

        - green + white
        - orange + white 
        map to: rollers on, not full
        
        - red fade for post game (based on team)
    */
    }
}