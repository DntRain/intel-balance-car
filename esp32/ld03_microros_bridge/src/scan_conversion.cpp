#include "scan_conversion.h"

#include <limits>

float scan_infinity() {
  return std::numeric_limits<float>::infinity();
}

void fill_scan_ranges(const std::array<ScanPoint, LD03Driver::kScanPointCount>& points,
                      std::array<float, LD03Driver::kScanPointCount>& ranges,
                      std::array<float, LD03Driver::kScanPointCount>& intensities) {
  for (size_t index = 0; index < points.size(); ++index) {
    if (points[index].valid && points[index].distance_mm > 0) {
      ranges[index] = static_cast<float>(points[index].distance_mm) / 1000.0F;
      intensities[index] = static_cast<float>(points[index].intensity);
    } else {
      ranges[index] = scan_infinity();
      intensities[index] = 0.0F;
    }
  }
}
