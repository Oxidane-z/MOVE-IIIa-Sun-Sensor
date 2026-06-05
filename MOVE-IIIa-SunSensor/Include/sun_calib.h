#ifndef SUN_CALIB_H
#define SUN_CALIB_H

// Per-sensor intrinsic calibration coefficients (centroid -> gnomonic u,v).
//
//   *** UNCALIBRATED IDENTITY PLACEHOLDER ***
//
// SUN_CALIB_PRESENT = 0 selects the linear pinhole baseline in
// compute_sun_vector()  ( u = x_c*K_GEOM, v = y_c*K_GEOM ) -- i.e. runtime
// behaviour is exactly the same as before any calibration.
//
// After a bench / sun calibration, OVERWRITE this file with the output of
//     python tools/calibration/fit_calibration.py --out-c sun_calib.h ...
// which emits  SUN_CALIB_PRESENT = 1  plus the polynomial tables and a
// trig-free  sun_apply_calib()  function, then rebuild.  No other source
// change is needed -- compute_sun_vector() picks up the polynomial via the
// #if SUN_CALIB_PRESENT switch.

#define SUN_CALIB_PRESENT 0

#endif // SUN_CALIB_H
