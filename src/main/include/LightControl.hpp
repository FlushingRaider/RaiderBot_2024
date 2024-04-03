#include <frc/AddressableLED.h>


extern std::array<frc::AddressableLED::LEDData, C_LedLength> Va_LED_outputBuffer;

void LightControl(
    TsENC_LightPatterns L_pattern);