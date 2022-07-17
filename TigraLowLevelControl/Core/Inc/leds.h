#ifndef __LEDS_H
#define __LEDS_H

typedef enum
{
    LEDS_EMERGENSY,
    LEDS_ROS,
    LEDS_ENABLE,
    LEDS_RIGHT,
    LEDS_LEFT
}Leds_t;

typedef enum
{
    LEDS_ON=0,
    LEDS_OFF=1
}Leds_state_t;

void ledsSet(Leds_t led,Leds_state_t state);
void ledsToggle(Leds_t led);

#endif