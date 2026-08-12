#ifndef CRT_MUSIC_H
#define CRT_MUSIC_H

#include <cstdint>

class Class_Music
{
public:
  /**
   * @brief 初始化蜂鸣器并阻塞播放开机音乐
   */
  void Play_Startup();

private:
  static void Play_Note(const float &__Frequency, uint32_t __Duration_Ms);
};

extern Class_Music Chariot_Music;

#endif
