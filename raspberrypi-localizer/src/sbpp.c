/*
 ============================================================================
 Name        : sbpp.c
 Author      : HJ
 Version     :
 Copyright   : no strings attached
 Description : Load image from the camera after background subtraction in 
               single bit per pixel format
 Created     : Nov 26, 2018
 ============================================================================
 */
#include "raspi-tex.h"
#include "raspi-tex-util.h"
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include "lodepng.h"
#include "led-detector.h"
#include "sbpp.h"


#define SAVE_PNG 0

/* Draw a scaled quad showing the the entire texture with the
 * origin defined as an attribute */
static RASPITEXUTIL_SHADER_PROGRAM_T sbpp_shader =
{
    .vertex_source =
    "attribute vec2 vertex;\n"
    "attribute vec2 top_left;\n"
    "varying vec2 texcoord;\n"
    "void main(void) {\n"
    "   texcoord = vertex + vec2(0.0, 1.0);\n"
    "   gl_Position = vec4(top_left + vertex, 0.0, 0.5);\n"
    "}\n",

    .uniform_names = {"current", "background", "luminance_thresh", "height"},
    .attribute_names = {"vertex", "top_left"},
};


static GLfloat varray[] =
{
   0.0f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f,
   1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
};

static const EGLint sbpp_egl_config_attribs[] =
{
   EGL_RED_SIZE,   8,
   EGL_GREEN_SIZE, 8,
   EGL_BLUE_SIZE,  8,
   EGL_ALPHA_SIZE, 8,
   EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
   EGL_NONE
};

unsigned char *data;

int image_counter = 0;

led_detector g_led_dectector;

SETUP_FPS

#if LOCALIZATION_DEBUG > 0


#ifdef LOC_ENABLE_SAVE_IMAGE
uint8_t  *image;
uint8_t  *image_data;
uint8_t  *image_array;
uint32_t image_array_index;

extern uint32_t led_detected;

static void bits_to_bytes_diff(uint8_t *d, uint8_t *im)
{
  int l = 0;

  for (int j = 0; j < FRAME_HEIGHT/16; j++) {
    for (int i = 0; i < FRAME_WIDTH*4; i+=4) {
      *((uint16_t*)(image_data + l)) = *((uint16_t*)(d + i + (j*FRAME_WIDTH*4)));
      l+=2;
    }
  }

  for (int y = 0; y < FRAME_HEIGHT; y++) {
    for (int x = 0; x < FRAME_WIDTH; x++) {
      uint32_t index = ((y/16) * (FRAME_WIDTH*2)) + (x*2) + ((y%16)>7);
      uint8_t bit = image_data[index] & (1 << (y&7));

      im[x + y*FRAME_WIDTH] = (!!bit)*255;
    }
  }

}

static void bits_to_bytes_diff_array(int i, uint8_t *im) {
  uint8_t *d = image_array + (i*((FRAME_HEIGHT * FRAME_WIDTH) / 4));
  bits_to_bytes_diff(d, im);
}



#endif /* LOC_ENABLE_SAVE_IMAGE */

uint32_t fq_endlist[128];

void adjust_fps(const double req_interval) 
{
    double current_interval, avg_interval;
    
    // Initialize for 25 FPS
    static double specific_interval = 40.0/1000.0;

    clock_gettime(CLOCK_REALTIME, &__gettime_now); 
    __time_difference.tv_sec = __gettime_now.tv_sec - __prev_time.tv_sec; 
    __time_difference.tv_nsec = __gettime_now.tv_nsec - __prev_time.tv_nsec; 
    current_interval = ((__time_difference.tv_sec * 1000000000.0) + __time_difference.tv_nsec)/1000000000.0;
    __frames++; 
    if ((__frames % __interval) == 0) {
      __time_difference.tv_sec = __gettime_now.tv_sec - __start_time.tv_sec; 
      __time_difference.tv_nsec = __gettime_now.tv_nsec - __start_time.tv_nsec; 
      avg_interval = ((__time_difference.tv_sec * 1000000000.0) + __time_difference.tv_nsec)/1000000000.0; 

      if (specific_interval > (25.0/1000.0))
        specific_interval += ((req_interval - (avg_interval/__frames))) / 2.0;
      else
        specific_interval = 40.0/1000.0;

      fprintf(stdout, "%s - FPS: %lf, AvgTime: %lf, led_queue_size: %d, frame_leds: %d, frame_ones: %d, frame_noise: %d, luminence_thresh: %f\r\n",__msg, __frames/avg_interval, 1000.0*(avg_interval/__frames), g_led_dectector.leds_queue_size, g_led_dectector.frame_leds, g_led_dectector.frame_ones, g_led_dectector.frame_noise, ((RASPITEX_STATE *)g_led_dectector.context)->luminence_thresh);
      fflush(stdout);
      __frames = 0; 
      __start_time = __gettime_now; 
    }

    clock_gettime(CLOCK_REALTIME, &__prev_time); 
}
#endif /* LOCALIZATION_DEBUG > 0*/

uint32_t  time_anomaly_counter = 0;
double    prev_time = 0.0;


static void process_framebuffer(RASPITEX_STATE *raspitex_state)
{
  static uint32_t current_frame = 0;
  double current_time, delta_time;
#ifdef LOC_ENABLE_SAVE_IMAGE  
  static int cc = 0;
#endif /* LOC_ENABLE_SAVE_IMAGE */
  glReadPixels(0,0,FRAME_WIDTH,FRAME_HEIGHT/16, GL_RGBA , GL_UNSIGNED_BYTE, data);

  if (raspitex_state->current_buf)
  {
#if LOCALIZATION_DEBUG > 0
#ifdef LOC_ENABLE_SAVE_IMAGE    
    if (cc == 0) 
    {
      uint8_t *d = image_array + (image_array_index*((FRAME_HEIGHT * FRAME_WIDTH) / 4));
      image_array_index = (image_array_index + 1) % raspitex_state->number_of_images;
      memcpy(d, data, ((FRAME_HEIGHT * FRAME_WIDTH) / 4));
    }
    
#endif /* LOC_ENABLE_SAVE_IMAGE */
#endif /* LOCALIZATION_DEBUG > 0 */

    current_time = raspitex_state->prev_buff_time;

    delta_time = current_time - prev_time;

    if (delta_time < 30 || delta_time > 50) 
    {
      time_anomaly_counter++;
    }
    else
    {
      time_anomaly_counter = 0;
    }
    
    if (time_anomaly_counter > 100) 
    {
      fprintf(stdout, "Missed - Time anomaly\r\n");
      fflush(stdout);
    }

    
    prev_time = current_time;
    
    g_led_dectector.is_new_frame = !!(raspitex_state->current_buf);
    led_detector_process(&g_led_dectector, data, current_time, current_frame++);
    
    
    if (raspitex_state->enable_dynamic_luminence) {
      if (g_led_dectector.frame_noise > (raspitex_state->on_pixels_in_frame*1.25)) {
        if (raspitex_state->luminence_thresh < LUMINENCE_THRESH_MAX) {
          raspitex_state->luminence_thresh += LUMINENCE_THRESH_DELTA;
        }
      } else if (g_led_dectector.frame_noise < (raspitex_state->on_pixels_in_frame*0.75)) {
        if (raspitex_state->luminence_thresh > LUMINENCE_THRESH_MIN) {
          raspitex_state->luminence_thresh -= LUMINENCE_THRESH_DELTA;
        }
      }
    }

#if LOCALIZATION_DEBUG > 0
#ifdef LOC_ENABLE_SAVE_IMAGE
    if ((g_led_dectector.led_identified == 1 || current_frame > 750)&& raspitex_state->save_image && current_frame > raspitex_state->save_image_warmup)
    {
      if (cc == 0) 
      {
        char fname[32];
        int i;
        cc++;
        for (i = 0; i < raspitex_state->number_of_images; i++) {
          unsigned error;

          led_detected = 0;

          printf("Saving Image\n");
          sprintf(fname, "%03d.png", i); 
          bits_to_bytes_diff_array(i, image);
          error = lodepng_encode_file(fname, image, FRAME_WIDTH, FRAME_HEIGHT, LCT_GREY, BITS_PER_BYTE);
          if(error) {
            printf("errorin saving frame: %d\n",error);
          }
        }
      }
      
    }
#endif
    adjust_fps(40.0/1000.0);
#endif /* LOCALIZATION_DEBUG > 0 */
  }
}


static char * loadshader(const char *filename) {
  FILE* f = fopen(filename, "rb");
  char *src;
  
  if (!f) {
    printf("shader file not found %s\n", filename);
    return 0;
  }

  fseek(f,0,SEEK_END);
  int sz = ftell(f);
  fseek(f,0,SEEK_SET);
  src = (char*)malloc(sz+1);
  fread(src,1,sz,f);
  src[sz] = 0; //null terminate it!
  fclose(f);

  return src;
}


/**
 * Creates the OpenGL ES 2.X context and builds the shaders.
 * @param raspitex_state A pointer to the GL preview state.
 * @return Zero if successful.
 */
int sbpp_init(RASPITEX_STATE *state)
{
  int rc;
  char *src;
  
  state->egl_config_attribs = sbpp_egl_config_attribs;
  rc = raspitexutil_gl_init_2_0(state);

  if (rc != 0)
    return rc;

  src = loadshader("glsl/binfragshader.glsl.c");
  sbpp_shader.fragment_source = src;

  rc = raspitexutil_build_shader_program(&sbpp_shader);
  
  GLCHK(glUseProgram(sbpp_shader.program));
  GLCHK(glUniform1i(sbpp_shader.uniform_locations[0], 0)); // tex unit
  GLCHK(glUseProgram(0));

  data = malloc(FRAME_WIDTH*FRAME_HEIGHT/2);
#ifdef LOC_ENABLE_SAVE_IMAGE
  image = malloc(FRAME_WIDTH*FRAME_HEIGHT*4);
  image_data = malloc(FRAME_WIDTH*FRAME_HEIGHT*4);
  image_array = malloc(FRAME_WIDTH*FRAME_HEIGHT*state->number_of_images);
  image_array_index = 0;
#endif /* LOC_ENABLE_SAVE_IMAGE */
  
  // Default parameters for ledDetector.
  led_detector_init(&g_led_dectector, state);
  
  g_led_dectector.context = state;

  START_FPS("Localizer", 100);

  return rc;
}

extern int bg_available;
int test = 0;

/**
 * Draws a 2x2 grid with each shell showing the entire MMAL buffer from a
 * different EGL image target.
 */
int sbpp_redraw(RASPITEX_STATE *raspitex_state)
{
  static int frame_counter = 0;

  frame_counter++;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

if (test == 0 ) 
{
    GLCHK(glUseProgram(sbpp_shader.program));
      GLCHK(glUniform1i(sbpp_shader.uniform_locations[0], 0)); // current unit
      GLCHK(glUniform1i(sbpp_shader.uniform_locations[1], 1)); // background unit

      GLCHK(glActiveTexture(GL_TEXTURE0));
      GLCHK(glEnableVertexAttribArray(sbpp_shader.attribute_locations[0]));
      GLCHK(glVertexAttribPointer(sbpp_shader.attribute_locations[0], 2, GL_FLOAT, GL_FALSE, 0, varray));
      GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, raspitex_state->texture[0]));
      
      GLCHK(glActiveTexture(GL_TEXTURE1));
      GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, raspitex_state->texture[1]));
      
      GLCHK(glActiveTexture(GL_TEXTURE0));
      GLCHK(glVertexAttrib2f(sbpp_shader.attribute_locations[1], -0.5f, 0.5f));

      GLCHK(glUniform1f(sbpp_shader.uniform_locations[3], (float)FRAME_HEIGHT));
      test = 1;
}
      GLCHK(glUniform1f(sbpp_shader.uniform_locations[2], raspitex_state->luminence_thresh));
      GLCHK(glDrawArrays(GL_TRIANGLES, 0, 6));
if (0) 
{      
      GLCHK(glDisableVertexAttribArray(sbpp_shader.attribute_locations[0]));
      GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0));
      GLCHK(glActiveTexture(GL_TEXTURE1));
      GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0));
    GLCHK(glUseProgram(0));

}
  if (bg_available)
    process_framebuffer(raspitex_state);

  return 0;
}

int sbpp_open(RASPITEX_STATE *state)
{
   state->is_ready = 1;
   return 0;
}
