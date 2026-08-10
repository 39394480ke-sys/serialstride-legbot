#ifndef SINGLE_LEG_CALIBRATION_H
#define SINGLE_LEG_CALIBRATION_H

/*
 * 2026-08-10 suspended single-leg calibration, expressed in motor raw
 * milliradians. Mechanical-contact endpoints are observations; commanded
 * targets remain inside the software limits.
 */

/* Mechanical-positive means motion toward leg extension. */
#define SINGLE_LEG_JOINT_A_DIRECTION 1
#define SINGLE_LEG_JOINT_B_DIRECTION (-1)

/* Temporary STAND is the arithmetic midpoint of CROUCH and EXTEND. */
#define SINGLE_LEG_JOINT_A_ZERO_MILLIRAD 1008
#define SINGLE_LEG_JOINT_B_ZERO_MILLIRAD (-1307)

/* Mechanical-contact captures. Do not command these endpoints. */
#define SINGLE_LEG_JOINT_A_MECHANICAL_MIN_MILLIRAD 662
#define SINGLE_LEG_JOINT_A_MECHANICAL_MAX_MILLIRAD 1354
#define SINGLE_LEG_JOINT_B_MECHANICAL_MIN_MILLIRAD (-1662)
#define SINGLE_LEG_JOINT_B_MECHANICAL_MAX_MILLIRAD (-951)

/* Commandable range after reserving 10% of captured travel at each end. */
#define SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD 731
#define SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD 1285
#define SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD (-1591)
#define SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD (-1022)

/* Low-speed suspended bring-up targets; not load-bearing pose definitions. */
#define SINGLE_LEG_JOINT_A_MID_CROUCH_MILLIRAD 835
#define SINGLE_LEG_JOINT_B_MID_CROUCH_MILLIRAD (-1129)
#define SINGLE_LEG_JOINT_A_MID_EXTEND_MILLIRAD 1181
#define SINGLE_LEG_JOINT_B_MID_EXTEND_MILLIRAD (-1484)

#endif
