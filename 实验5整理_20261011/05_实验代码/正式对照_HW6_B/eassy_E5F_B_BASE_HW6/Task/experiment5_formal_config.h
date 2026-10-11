#ifndef E5_FORMAL_CONFIG_H
#define E5_FORMAL_CONFIG_H
/* Condition B formal batch. No added mass, 2 mm out/back.
 * Only the RIGHT bound tightens from 1200 to 800 counts versus frozen A.
 * No gain, trajectory, learning, speed or fault threshold changes.
 * Independent formal firmware; common control and protection unchanged.
 * Configured load is a label, not a measured mass. */
#define E5_SELECTED_METHOD 10U
#define E5_SELECTED_BUILD 63010U
#define E5_SELECTED_LOAD_G 0U
#define E5_SELECTED_LIMIT_LEFT 1200
#define E5_SELECTED_LIMIT_RIGHT 800
#define E5_FORMAL_TELEMETRY_CHANNELS 107U
#endif
