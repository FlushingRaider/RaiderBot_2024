#include <frc/AddressableLED.h>

extern std::array<frc::AddressableLED::LEDData, C_LedLength> Va_LED_outputBuffer;

void LightControl(
    std::optional<frc::DriverStation::Alliance> L_Alliance,
    double L_MatchTimeRemaining);