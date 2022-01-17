/*
 ============================================================================
 Name        : led.c
 Author      : HJ
 Version     :
 Copyright   : no strings attached
 Description : Receive LED ID using Manchester Encoding 
 Checksum    : CRC with polynomial: x^4 + x^2 + x + 1
 Created     : Nov 26, 2018
 Updated     : August 20, 2020
 ============================================================================
 */

#include <sys/types.h>
#include <pthread.h>
#include "configurations.h"
#include "led.h" 

void led_init_vals(led *l, uint16_t x, uint16_t y, uint16_t one_zero_thresh, uint16_t led_radius, uint16_t frame_number, double frame_time, uint32_t area)
{
  l->x = x;
  l->y = y;
  l->area = area;
  l->id = 0;
  l->current_bit_start_time = frame_time;
  l->transmission_start_time = frame_time;
  l->prev_state_end_time = frame_time;
  l->one_zero_thresh = one_zero_thresh;
  l->led_radius = led_radius;
  l->area_sum = one_zero_thresh;
  l->ones = 1;
#if DEBUG_LED  
  l->debug_buffer_index = 0;
  l->debug_prev_bit_index = 0;
  for (int i = 0; i < LED_BUFFER_LENGTH*3; i++) {
    l->debug_prev_bit[i] = 8;
  }
  memset(l->debug_buffer_indexes, 0, LED_BUFFER_LENGTH*3*4);
  
#endif
  l->start_frame_index = frame_number;
  l->is_first_frame = 1;
  l->raw_data = 0;
}

led* led_create_vals(led_detector *ld, uint16_t x, uint16_t y)
{
  led *l = (led*)malloc(sizeof(led));
  led_init_vals(l, x, y, ld->one_zero_thresh, ld->led_radius, ld->frame_number, ld->frame_time, ld->area);
  
  return l;
}

#define GENPOLY 0x0017 /* x^4 + x^2 + x + 1 */

uint16_t led_calculate_checksum(uint16_t val)
{
  uint16_t i;

  i=1;
  while (val>=16) { /* >= 2^4, so degree(val) >= degree(genpoly) */
    if (((val >> (16-i)) & 1) == 1)
    val ^= GENPOLY << (12-i); /* reduce with GENPOLY */
    i++;
  }
  return val;
}

/* TODO: Replace with a lookup table based bit counter. */
typedef struct bit_access_field_t {
  uint8_t a0:1;
  uint8_t a1:1;
  uint8_t a2:1;
  uint8_t a3:1;
  uint8_t a4:1;
  uint8_t a5:1;
  uint8_t a6:1;
  uint8_t a7:1;
} bit_access_field;

uint32_t led_get_roi_sum(led *l, uint8_t *frame, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
  uint32_t sum = 0;
  bit_access_field *sa;

  for (uint32_t i = y1; i < y2 /*&& sum <= l->one_zero_thresh*/; i+=8)
  {
    for (uint32_t j = x1; j < x2 /*&& sum <= l->one_zero_thresh*/; j++)
    {
      uint32_t index = (i/16 * (FRAME_WIDTH*2)) + j*2 + (i%16>7);
      sa = (bit_access_field*) (frame + index);
      sum += (sa->a0 + sa->a1 + sa->a2 + sa->a3 + sa->a4 + sa->a5 + sa->a6 + sa->a7);
    }
  }

  return sum;
}

/*
 Process LED bits sent using Manchester encoding.

*/
uint8_t led_process(led *l, uint8_t *frame, double frame_time, uint8_t is_new_frame)
{
  uint32_t sum;
  uint32_t x1, y1, x2, y2;

  uint8_t current_frame_state;
  uint8_t status = 0;
  uint32_t data = 0;
  uint32_t checksum = 0;
  uint8_t is_state_flip;

  uint8_t state_based_end_transmission = 0;
  uint8_t bit_based_end_transmission = 0;
  
  double state_elapsed_time;

  /* If LED ID is already extracted, just return 1, indicating that no further action is needed. */
  if (l->id)
    return 1;
 

  /* Bounding box for the LED. Calculate using a square instead of a circle for the sake of performance. */
  x1 = (l->x > l->led_radius) ? (l->x - l->led_radius) : 0;
  y1 = (l->y > l->led_radius) ? (l->y - l->led_radius) : 0;

  x2 = ((l->x + l->led_radius) < FRAME_WIDTH  ) ? (l->x + l->led_radius) :  (FRAME_WIDTH);
  y2 = ((l->y + l->led_radius) < FRAME_HEIGHT ) ? (l->y + l->led_radius) :  (FRAME_HEIGHT);

  /* Get the total number of 1's in the bounding box. */
  sum = led_get_roi_sum(l, frame, x1, y1, x2, y2);
  
  /*Threshold the number of 1's */
  uint32_t thresh = (l->one_zero_thresh);
  current_frame_state = sum > thresh;

  /* Flag state flip as compared to previous frame. */
  is_state_flip = (l->prev_frame_state != current_frame_state);
  
  bit_based_end_transmission = (frame_time - l->current_bit_start_time ) > (BIT_TRANSFER_TIME + 2*FRAME_TRANSFER_TIME_F + 10 );
  
  state_elapsed_time = frame_time - l->prev_state_end_time;

  if (l->prev_frame_state) 
  {
    state_based_end_transmission = state_elapsed_time > (BIT_TRANSFER_TIME + 2*FRAME_TRANSFER_TIME_F + 10 );
  } 
  else 
  {
    state_based_end_transmission = state_elapsed_time > (BIT_TRANSFER_TIME + FRAME_TRANSFER_TIME_F + 10 );
  }

  if (is_state_flip)
  {
    if (!state_based_end_transmission && l->prev_frame_state) 
    {
      state_based_end_transmission = state_elapsed_time < (2*FRAME_TRANSFER_TIME_F - 10 );
    } 
    else 
    {
      // It is possible to have a single zero frame
    }

    l->prev_state_end_time = frame_time;
  }

  state_based_end_transmission = (state_based_end_transmission && (l->ones > 3));

  /* Force end of transmission if the current bit did not flip for three times the BIT_TRANSFER_TIME */
  /* Can be determined after 2*BIT_TRANSFER_TIME, but use 2.5 to compensate for processing delay. */
  uint8_t is_end_transmission = (!is_state_flip && bit_based_end_transmission) || state_based_end_transmission;

  /* Init status value to error in case of end of transmission. This will make sure that the LED node is removed from queue */
  if (is_end_transmission) 
  {
    status = 2;
  }

#if DEBUG_LED
  if (l->debug_buffer_index < (LED_BUFFER_LENGTH*3)) 
  {
    l->debug_buffer[l->debug_buffer_index] = sum;
    l->debug_buffer_time[l->debug_buffer_index] = (frame_time - l->current_bit_start_time);//frame_time;
    l->debug_buffer_indexes[l->debug_buffer_index] = thresh;//current_frame_state;
  }
  else
  {
    fprintf(stdout,"overflow");
  }
#endif
  
  /* Process bit flip. */
  /* Force process if end of transmission. */
  /* Force not to process if start of transmission. */
  if (!l->is_first_frame && (is_state_flip || bit_based_end_transmission)) 
  {
    double elapsed_time = frame_time - l->current_bit_start_time;
    uint8_t is_data = 0;
    
    /* Handle 1 and 0 differently as a 1 can overflow into a zero bit. */
    /* The elapsed time determines if it is a data flip or an intermediate flip. */
    if (current_frame_state == 1) {
      is_data = elapsed_time > BIT_TRANSFER_TIME/2;
    } else {
      is_data = elapsed_time > (BIT_TRANSFER_TIME/2 + FRAME_TRANSFER_TIME_F);
    }

    /* If it is a data flip, record the data and adjust counters. */
    /* Don't wait for elapsed time if its the very first flip.*/
    if ((is_state_flip || bit_based_end_transmission) && ((l->raw_data == 0) || is_data)) {
      l->raw_data <<= 1;
      l->raw_data |= !current_frame_state;
      l->current_bit_start_time = frame_time;
#if DEBUG_LED
      if (l->debug_buffer_index < (LED_BUFFER_LENGTH*3)) 
      {
        l->debug_prev_bit[l->debug_buffer_index] = !current_frame_state;
      }
#endif
    } else {
      // ignore
    }


  }

  if (current_frame_state) {
    l->area_sum += sum;
    l->ones++;
  }
  
  /* If the message has a preemble bit at the start  */
  if (l->raw_data & 0x100000) {
    
    /* Decode data */
    data = (l->raw_data >> 4) & 0xFFFF;
    
    /* Decode checksum */
    checksum = l->raw_data & 0xF;

    /* Verify checksum. */
    if (data && led_calculate_checksum(data) == checksum) {
      l->id = data;
      status = 1;
    }
  }

#if DEBUG_LED
    if (status == 2 && l->raw_data) 
    {
      if (l->debug_buffer_index > 80) {
        for (int i=0;i<l->debug_buffer_index;i++) {
          fprintf(stdout, "%4.0f %04d %04d %d\n", l->debug_buffer_time[i], l->debug_buffer[i], l->debug_buffer_indexes[i], l->debug_prev_bit[i]);
        }
      }
      fprintf(stdout, "    status: %d - (%d, %d) - message frames: %d, state_based_end_transmission: %d\n", status, l->x, l->y, l->debug_buffer_index, state_based_end_transmission);
      fprintf(stdout, "    raw data: 0x%04x, data: 0x%04x, checksum: 0x%04x, calculated checksum: 0x%04x\n", 
        l->raw_data, 
        (l->raw_data >> 4) & 0xFFFF,
        l->raw_data & 0xF, 
        led_calculate_checksum((l->raw_data >> 4) & 0xFFFF));

      fflush(stdout);
    }
    
    l->debug_buffer_index++;
#endif


  l->prev_frame_state = current_frame_state;
  l->prev_frame_time = frame_time;
  l->is_first_frame = 0; 

  return status;
}
