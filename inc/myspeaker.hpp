#ifndef SPEAKER
#define SPEAKER

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <sys/wait.h>
#include <alsa/asoundlib.h>

void speakAndSave(const char *text, const char *wavfile);

void playWav(const char *wav_file, double volume);

void speak(const char*);

#endif SPEAKER