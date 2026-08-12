#include "crt_music.h"

#include "bsp_buzzer.h"
#include "stm32h7xx_hal.h"

namespace
{
constexpr float Startup_Loudness = 0.5f;
}

Class_Music Chariot_Music;

void Class_Music::Play_Note(const float &__Frequency, uint32_t __Duration_Ms)
{
  BSP_Buzzer.Set_Sound(__Frequency, Startup_Loudness);

  // HAL_Delay(n) lasts n + 1 ms in this project.
  if (__Duration_Ms > 0U)
  {
    HAL_Delay(__Duration_Ms - 1U);
  }
}

void Class_Music::Play_Startup()
{
  BSP_Buzzer.Init(4000.0f, 0.0f);

  // Sequence copied from StandardRobotpp/application/music/music_start.c.
  Play_Note(0.0f, 2U);
  Play_Note(BUZZER_FREQUENCY_C4, 200U);
  Play_Note(0.0f, 3U);
  Play_Note(BUZZER_FREQUENCY_E5, 200U);
  Play_Note(0.0f, 3U);
  Play_Note(BUZZER_FREQUENCY_G6, 200U);
  Play_Note(0.0f, 15U);
  Play_Note(BUZZER_FREQUENCY_C6, 500U);

  BSP_Buzzer.Set_Sound(0.0f, 0.0f);
}
