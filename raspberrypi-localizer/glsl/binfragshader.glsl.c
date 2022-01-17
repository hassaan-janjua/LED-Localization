#extension GL_OES_EGL_image_external : require

uniform samplerExternalOES current;
uniform samplerExternalOES background;
uniform float luminance_thresh;
uniform float height;

varying vec2 texcoord;

void main(void) {
  
  float roundedy = (floor((texcoord.y * height)/16.0) / height) * 16.0;
  
  vec4 c;
  vec4 lv, rv;

  float thresh = luminance_thresh; 
  float y, y_inc, x_inc;
  float diff = 0.0;
  float diffbit = 1.0/256.0;
  int count = 0;
  vec2 coord;

  y = roundedy;
  y_inc = 1.0/height;

  coord = vec2(texcoord.x, y);

  for (int j = 0; j < 8; j++)
  {

    lv = texture2D(current, coord);
    rv = texture2D(background, coord);
    y += y_inc;
    coord = vec2(texcoord.x, y);

    if (lv.r > thresh || abs(lv.r - rv.r) > thresh)
    {
      diff = diff + diffbit;
      count++;
    }

    diffbit += diffbit;
  }

  c[0] = diff + 1.0/512.0;
  diff = 0.0;
  diffbit = 1.0/256.0;
  
  for (int j = 8; j < 16; j++) {
    lv = texture2D(current, coord);
    rv = texture2D(background, coord);
    y += y_inc;
    coord = vec2(texcoord.x, y);

    if (lv.r > thresh || abs(lv.r - rv.r) > thresh)
    {
      diff = diff + diffbit;
      count++;
    }

    diffbit += diffbit;
  }

  c[1] = diff + 1.0/512.0;
  
  if (count < 2)
  {
    c[0] = 0.0;
    c[1] = 0.0;
  }

  c[2] = 0.0;
  c[3] = 0.0;
  gl_FragColor = c;
  
}
