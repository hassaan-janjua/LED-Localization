
/*
 ============================================================================
 Name        : led-detector.c
 Author      : HJ
 Version     :
 Copyright   : no strings attached
 Description : LED Detection logic
 ============================================================================
 */

#include <unistd.h>
#include <pthread.h>
#include "led-detector.h"

#ifdef LOC_ENABLE_SAVE_IMAGE
uint32_t led_detected;
#endif /* LOC_ENABLE_SAVE_IMAGE */


uint32_t led_found = 0;

void led_detector_init(led_detector *ld, RASPITEX_STATE *state)
{
  ld -> is_first_frame = 1;
  ld -> minx = 0;
  ld -> miny = 0;
  ld -> maxx = 0;
  ld -> maxy = 0;
  ld -> area = 0;
  ld -> leds = NULL;
  ld -> leds_queue_size = 0;
  ld -> led_find_radius = state->led_find_radius;
  ld -> led_blob_size = state->led_blob_size;
  ld -> led_radius = state->led_radius;
  ld -> one_zero_thresh = state->led_one_zero_thresh;
  ld -> led_identified = 0;
}

void led_detector_destroy(led_detector *ld)
{
  queue_clean(& ld -> leds);
}


void led_detector_flood_check(led_detector *ld, uint16_t x, uint16_t y)
{
  uint32_t index = ((y/16) * (FRAME_WIDTH*2)) + (x*2) + ((y%16)>7);

  uint8_t bit = ld -> prev_bit_frame[index] & (1 << (y&7));

  if (bit)
  {
    ld -> prev_bit_frame[index] &= ~bit;
    //ld -> prev_bit_frame[index] = 0;
    ld->area++;
    ld->frame_ones++;
    
    /* Return if already found 1's in 1/2 of the total area */
    //if (ld -> area > ((ld -> led_radius) * 2 * (ld -> led_radius) * 2 )/2)
    //  return;
    if (ld -> area > 10000)
      return;

    if (x < ld -> minx)
      ld -> minx = x;
    else if (x > ld -> maxx)
      ld -> maxx = x;
    if (y < ld -> miny)
      ld -> miny = y;
    else if (y > ld -> maxy)
      ld -> maxy = y;


    if (y > 0)
    {
      if (x > 0)
      {
        // top left
        led_detector_flood_check(ld, x - 1, y - 1);
      }

      // top centre
      led_detector_flood_check(ld, x , y - 1);

      if (x < (FRAME_WIDTH - 1))
      {
        // top right
        led_detector_flood_check(ld, x + 1, y - 1);
      }
    }

    if (x > 0)
    {
      // left
      led_detector_flood_check(ld, x - 1, y);
    }

    if (x < (FRAME_WIDTH - 1))
    {
      // right
      led_detector_flood_check(ld, x + 1, y);
    }

    if (y < (FRAME_HEIGHT - 1))
    {
      if (x > 0)
      {
        // bottom left
        led_detector_flood_check(ld, x - 1, y + 1);
      }
      // bottom centre
      led_detector_flood_check(ld, x, y + 1);

      if (x < (FRAME_WIDTH - 1))
      {
        // bottom right
        led_detector_flood_check(ld, x + 1, y + 1);
      }
    }
  }

}

void led_detector_check_and_add_led(led_detector *ld, uint16_t x, uint16_t y)
{
  ld -> minx = x;
  ld -> maxx = x;
  ld -> miny = y;
  ld -> maxy = y;
  ld -> area = 0;

  led_detector_flood_check(ld, x, y);

  x = (ld -> minx + ld -> maxx)/2;
  y = (ld -> miny + ld -> maxy)/2;

  if (ld -> area > ld -> led_blob_size)
  {
    led_found = 0;
    led *found = led_detector_find_led(ld, x, y);
    ld->frame_leds++;
    if (!found)
    {
      led *l = led_create_vals(ld, x, y);
      led_detector_add_led(ld, l);
    } else {
      found->x = (x + found->x)/2;
      found->y = (y + found->y)/2;      
    }
  } else {
    ld -> frame_noise++;
  }

}

typedef struct frame_info_t {
  double frame_time;
  uint32_t frame_number;
} frame_info;

pthread_t thread;
uint8_t diff_frame_queue[128][FRAME_HEIGHT * FRAME_WIDTH / 8];
frame_info frame_info_queue[128];
uint32_t fq_start = 0;
uint32_t fq_end = 0;
uint32_t fq_size = 0;
uint8_t keep_alive;

uint32_t led_detector_process_internal(led_detector *ld, uint8_t *diffFrame, frame_info *finfo);

void* led_detector_process_worker(void *args)
{
  led_detector *ld = (led_detector *)args;
  while (fq_size || keep_alive)
  {
    if (fq_size)
    {
      __sync_fetch_and_sub(&fq_size, 1);
      led_detector_process_internal(ld, diff_frame_queue[fq_end], &frame_info_queue[fq_end]);
      fq_end = (fq_end + 1) & 127;
    } else {
      usleep(1000);
    }
  }

  return NULL;
}

void led_detector_process_worker_thread(led_detector *ld)
{
  pthread_create(&thread, NULL, led_detector_process_worker, ld);
}

uint32_t led_detector_process(led_detector *ld, uint8_t *bFrame, double frame_time, uint32_t frame_number)
{
  if (fq_size < 127) {
    int l = 0;
    for (int j = 0; j < FRAME_HEIGHT/16; j++) {
      for (int i = 0; i < FRAME_WIDTH*4; i+=4) {
        diff_frame_queue[fq_start][l] = bFrame[i + (j*FRAME_WIDTH*4)];
        diff_frame_queue[fq_start][l+1] = bFrame[i + 1 + (j*FRAME_WIDTH*4)];
        l+=2;
      }
    }

    frame_info_queue[fq_start].frame_time = frame_time;
    frame_info_queue[fq_start].frame_number = frame_number;
    fq_start = (fq_start + 1) & 127;
    __sync_fetch_and_add(&fq_size, 1);
  }
  else
  {
    fprintf(stdout, "Missed %d\n", fq_start);
    fflush(stdout);
  }
#ifndef __MINGW32__
  if (keep_alive == 0) {
    keep_alive = 1;
    led_detector_process_worker_thread(ld);
  }
#else
  led_detector_process_worker(ld);
#endif
  return 0;
}

#if DEBUG_LUMINENCE_THRESH
uint32_t frame_count = 0;
uint32_t total_ones[32];
#endif /* DEBUG_LUMINENCE_THRESH */

void led_detector_detect_leds(led_detector *ld, uint8_t *bFrame)
{
  const uint32_t bitframeLength =  (FRAME_HEIGHT * FRAME_WIDTH)/8;
  uint32_t *curr, *worker;
  //uint32_t *end;
  if (ld -> is_first_frame) {
    memcpy(ld -> prev_bit_frame, bFrame, bitframeLength);
    ld -> is_first_frame = 0;
    return;
  }
  
  curr = (uint32_t*) bFrame;
  worker = (uint32_t*) ld -> prev_bit_frame;

  memcpy(worker, curr, bitframeLength);

  ld -> frame_ones = 0;
  ld -> frame_leds = 0;
  ld -> frame_noise = 0;

  for (uint32_t i = 0; i < FRAME_HEIGHT; i+=16)
  {
    for (uint32_t j = 0; j < FRAME_WIDTH*2; j+=4)
    {
      uint32_t word = *worker;
      if (word)
      {
        for (uint32_t k = 0; k < 32; k++ )
        {
          word = *worker;
          if (word & (1 << k ))
          {
            led_detector_check_and_add_led(ld, j/2 + k/16, i + k%16);
          }
        }
      }
      worker++;
    }
  }
#if DEBUG_LUMINENCE_THRESH
  if (frame_count == 32) {
    for (int i = 0; i < 32; i++) {
      fprintf(stdout, "%04d ", total_ones[i]);
    }
    fprintf(stdout, "\n");
    fflush(stdout);
    frame_count = 0;
  }
  total_ones[frame_count] = ld->frame_ones;
  frame_count++;
#endif /* DEBUG_LUMINENCE_THRESH */
  /* restore the original frame */
  //memcpy(ld -> prev_bit_frame, bFrame, bitframeLength);
}

uint32_t led_detector_process_internal(led_detector *ld, uint8_t *diffFrame, frame_info *finfo)
{
  uint32_t count = 0;
  ld -> frame_time = finfo->frame_time;
  ld -> frame_number = finfo->frame_number;
  led_detector_detect_leds(ld, diffFrame);
#ifdef LOC_ENABLE_SAVE_IMAGE  
  led_detected = 0;
#endif /* LOC_ENABLE_SAVE_IMAGE */
  for(queue_node **n = &ld->leds; *n; ) {
    led *l = (led*)(*n) -> data;
#ifdef LOC_ENABLE_SAVE_IMAGE    
    led_detected = 1;
#endif /* LOC_ENABLE_SAVE_IMAGE */
    if (! (l->id))
    {
      uint8_t valid = led_process(l, diffFrame, finfo->frame_time, ld->is_new_frame);
      if (valid)
      {
        if (valid == 1) {
          ld->led_identified = 1;
          fprintf(stdout, "%d: (%d, %d, %d) - Area: %d, Average Area: %d, Frame: %d, Frame Noise: %d, qsize: %d\n", l->id & LED_DATA_MASK, l->id, l->x, l->y, l->area, l->area_sum/l->ones, l->start_frame_index, ld -> frame_noise, ld->leds_queue_size);
          
          fflush(stdout);
          count++;
        }
        free(l);
        queue_remove(n);
        ld -> leds_queue_size -= 1;
      }
      else
      {
        n = &((*n) -> next);
      }
    }
  }
  
  return count;
}

uint8_t led_detector_add_led(led_detector *ld, led* d)
{
    queue_add(&ld->leds, d);
    ld -> leds_queue_size += 1;
    return 0;
}

#define min_clamp(v, m) (((v) <= (m))?0:((v)-(m)))

led* led_detector_find_led(led_detector *ld, uint16_t x, uint16_t y)
{
  led* found = 0;
  for(queue_node **n = &ld->leds; *n; n = &((*n) -> next))
  {
    led* l = (led*) (*n) -> data;
    if ( (l -> x + ld->led_find_radius) >= x &&
         min_clamp(l -> x, ld->led_find_radius) <= x &&
         (l -> y + ld->led_find_radius) >= y &&
         min_clamp(l -> y, ld->led_find_radius) <= y )
    {
      found = l;
      break;
    }
  }
  return found;
}

