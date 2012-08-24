#ifndef LEVEL_H
#define LEVEL_H

struct Level {
  bool search_space_found;

  float scale;

  int image_height;
  int image_width;
  int orig_window_height;

  int orig_start_y;
  int orig_end_y;
  int resized_start_y;
  int resized_end_y;
};
#endif
