#ifndef GLOBAL_INFO_H
#define GLOBAL_INFO_H

// global variables to store tag info from the topic /tag_info
extern int g_enter_target_areas;
extern int g_small_box;
extern int g_stay_still;
extern double g_box_tag_position[3];

// global variables to store human motion info from the topic /human_motion_info
extern int g_predict_steady_box_size;
extern int g_predict_steady_box_position;
extern int g_predict_small_box;
extern int g_predict_box_on_left;

#endif  // GLOBAL_INFO_H
