/*********************************************************************************************************************
* File: single_gap_detector.h
* Description: Bounded grayscale detector for the two-cone validation gate.
*********************************************************************************************************************/

#ifndef _single_gap_detector_h_
#define _single_gap_detector_h_

#include "single_gap_types.h"

uint8 single_gap_detector_process(const uint8 *pixels,
                                  uint16 width,
                                  uint16 height,
                                  uint16 stride,
                                  uint32 sequence,
                                  uint32 capture_ms,
                                  single_gap_observation_struct *observation);

#endif
