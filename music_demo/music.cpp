#include "music.h"

unsigned long previousMillis = 0;        // will store last time LED was updated
int thisNote = 0;
bool play = true;
int num = 0;
int noteDuration = 0;

void playSong(unsigned long currentMillis, int melody[], int noteDurations[]) {
  if ((currentMillis - previousMillis >= noteDuration) && play) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    noteDuration = 1000 / noteDurations[thisNote];
    tone(speakerPin, melody[thisNote], noteDuration);
    
    if(melody[thisNote+1] && noteDurations[thisNote+1]){
      thisNote++;
    }else{
      play = false;
    }
  } else if (((currentMillis - previousMillis < noteDuration) && (currentMillis - previousMillis >= 0.9*noteDuration) && play) || 
  ((currentMillis - previousMillis >= noteDuration) && !play)) {
    noTone(speakerPin);
  }
}
