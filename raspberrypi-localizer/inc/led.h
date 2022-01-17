/*
 * Led.h
 *
 *  Created on: Feb 13, 2020
 *      Author: Hassaan
 */

#ifndef LED_H_
#define LED_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "configurations.h"
#include "led-detector.h"

#define DEBUG_LED 0

typedef struct led_t
{
#if DEBUG_LED  
  uint32_t debug_prev_bit[LED_BUFFER_LENGTH*3];
  uint32_t debug_buffer[LED_BUFFER_LENGTH*3];
  double   debug_buffer_time[LED_BUFFER_LENGTH*3];
  uint32_t debug_buffer_indexes[LED_BUFFER_LENGTH*3];
  uint32_t debug_buffer_index;
  uint32_t debug_prev_bit_index;
#endif
  uint32_t raw_data;
  uint8_t  prev_frame_state;
  uint8_t  last_flip_was_data;
  uint8_t  is_first_frame;
  double   transmission_start_time;
  double   current_bit_start_time;
  double   prev_state_end_time;
  double   prev_frame_time;
  uint32_t start_frame_index;

  uint16_t x;
  uint16_t y;
  uint16_t id;

  uint16_t one_zero_thresh;
  uint16_t led_radius;
  uint32_t area;
  uint32_t area_sum;
  uint32_t ones;
} led;

struct led_detector_t;
typedef struct led_detector_t led_detector;

void      led_init_vals(led *l, uint16_t x, uint16_t y, uint16_t one_zero_thresh, uint16_t led_radius, uint16_t frame_number, double frame_time, uint32_t area);
led*      led_create_vals(led_detector *ld, uint16_t x, uint16_t y);
uint8_t   led_is_packet_valid(led *l);
uint8_t   led_process(led *l, uint8_t *frame, double frame_time, uint8_t is_new_frame);
uint16_t  led_calculate_checksum(uint16_t data);
uint32_t  led_append_to_raw_data_buffer(led *l, uint8_t v, double frame_time);
uint32_t  led_get_roi_sum(led *l, uint8_t *frame, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);

#endif /* LED_H_ */
