#pragma once

#include <array>

#include "ld03_driver.h"

float scan_infinity();
void fill_scan_ranges(const std::array<ScanPoint, LD03Driver::kScanPointCount>& points,
                      std::array<float, LD03Driver::kScanPointCount>& ranges,
                      std::array<float, LD03Driver::kScanPointCount>& intensities);
