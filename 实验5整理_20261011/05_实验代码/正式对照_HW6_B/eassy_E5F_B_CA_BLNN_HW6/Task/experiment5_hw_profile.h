#ifndef E5_HW_PROFILE_H
#define E5_HW_PROFILE_H
/* Independent, PROVISIONAL real-hardware profile HW6, not frozen BLNN v3.
 * User authorized re-evaluating the hard rpm threshold after HW5 fault 512.
 * ONLY hard rpm changes from 60 to 120; this is NOT a certified safe speed.
 * HW5 unloaded peak was 74; older loaded runs exceeded 200 motor rpm.
 * Different load/stroke: historical records do not validate unloaded 120.
 * Preserve filtered core feedback, 10-30 taper and >30 active braking,
 * gain floors, disabled feedforward and every other safety threshold.
 * Shared candidate profile for BASE/BLNN/CA_BLNN; no method-specific tuning.
 * Do not export this change into the frozen simulation or old firmware. */
#define E5_HW_PROFILE_ID 6U
#define E5_HW_HARD_RPM 120
#define E5_HW_K2_Y_MIN 12.0
#define E5_HW_K2_ALPHA_MIN 1.0
#define E5_HW_COULOMB_FF 0.0
#define E5_HW_SYNC_STIFFNESS_FF 0.0
#define E5_HW_TAPER_RPM 10
#define E5_HW_SOFT_RPM 30
#define E5_HW_BRAKE_COUNTS_PER_RPM 10
#define E5_HW_BRAKE_LIMIT 200
#endif
