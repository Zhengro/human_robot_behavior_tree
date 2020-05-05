#include <global_info.h>

// global variables to store tag info from the topic /tag_info
int g_enter_target_areas;
int g_small_box;
int g_stay_still;
double g_box_tag_position[3];

// global variables to store human motion info from the topic /human_motion_info
int g_predict_steady_box_size;
int g_predict_steady_box_position;
int g_predict_small_box;
int g_predict_box_on_left;

