#include <array>

#include <frc/AddressableLED.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Const.hpp"
#include "ADAS_DJ.hpp"
#include "Amp.hpp"
#include "SpeakerCntrl.hpp"

int firstPixelHue = 0;

bool topHit = false;

double timer = 0;
double shooterTimer = 0;

int color = 0;

bool timerToggle = false;

std::array<frc::AddressableLED::LEDData, C_LedLength> Va_LED_outputBuffer;

void LightControl(
    std::optional<frc::DriverStation::Alliance> L_Alliance,
    double L_MatchTimeRemaining)
{

    TsENC_LightPatterns L_pattern = E_LED_FADEALLIANCE;

    if (VeADAS_e_Amp_SchedState == E_DJ_Amp_Intake ||
        VsAmp_s_Sensors.b_Amp_ObjDetected == true)
    {
        if (VsAmp_s_Sensors.b_Amp_ObjDetected == true)
        {
            L_pattern = E_LED_GREEN;
        }
        else
        {
            L_pattern = E_LED_GREENSTROBE;
        }
    }
    else if (VeSPK_e_AttndState == E_SPK_Ctrl_PreScore || VeSPK_e_AttndState == E_SPK_Ctrl_Score)
    {
        shooterTimer += C_ExeTime;
        if (shooterTimer >= 1.0)
        {
            L_pattern = E_LED_PINKSTROBE;
        }
    }
    else if (VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Intake ||
             VsSPK_s_Sensors.b_NoteDetected == true)
    {
        if (VsSPK_s_Sensors.b_NoteDetected == true)
        {
            L_pattern = E_LED_ORANGE;
        }
        else
        {
            L_pattern = E_LED_ORANGESTROBE;
        }
    }
    else if(L_MatchTimeRemaining <= 30.0 && L_MatchTimeRemaining > 0){
        L_pattern = E_LED_RAINBOW;
    }
    else{
        L_pattern = E_LED_FADEALLIANCE;
        shooterTimer = 0;
    }
    // frc::SmartDashboard::PutNumber("led pattern", (int)L_pattern);

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
    case E_LED_FADEALLIANCE:

        for (int i = 0; i < C_LedLength; i++)
        {
            if (L_Alliance == frc::DriverStation::Alliance::kBlue)
            {
                Va_LED_outputBuffer[i].SetRGB(0, 0, color);
            }
            else if (L_Alliance == frc::DriverStation::Alliance::kRed)
            {
                Va_LED_outputBuffer[i].SetRGB(color, 0, 0);
            }
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

    case E_LED_ORANGE:
        for (int i = 0; i < C_LedLength; i++)
        {
            Va_LED_outputBuffer[i].SetRGB(255, 50, 0);
        }
        break;

    case E_LED_ORANGESTROBE:
        for (int i = 0; i < C_LedLength; i++)
        {
            if (timerToggle)
            {
                Va_LED_outputBuffer[i].SetRGB(255, 50, 0);
            }
            else
            {
                Va_LED_outputBuffer[i].SetRGB(0, 0, 0);
            }
        }

        if (timer >= 0.1)
        {
            timerToggle = !timerToggle;
            timer = 0;
        }

        timer += C_ExeTime;

        break;

    case E_LED_GREEN:
        for (int i = 0; i < C_LedLength; i++)
        {
            Va_LED_outputBuffer[i].SetRGB(0, 255, 0);
        }
        break;

    case E_LED_GREENSTROBE:
        for (int i = 0; i < C_LedLength; i++)
        {
            if (timerToggle)
            {
                Va_LED_outputBuffer[i].SetRGB(0, 255, 0);
            }
            else
            {
                Va_LED_outputBuffer[i].SetRGB(0, 0, 0);
            }
        }

        if (timer >= 0.1)
        {
            timerToggle = !timerToggle;
            timer = 0;
        }

        timer += C_ExeTime;

        break;
    case E_LED_PINKSTROBE:
        for (int i = 0; i < C_LedLength; i++)
        {
            if (timerToggle)
            {
                Va_LED_outputBuffer[i].SetRGB(178, 102, 255);
            }
            else
            {
                Va_LED_outputBuffer[i].SetRGB(0, 0, 0);
            }
        }

        if (timer >= 0.1)
        {
            timerToggle = !timerToggle;
            timer = 0;
        }

        timer += C_ExeTime;

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