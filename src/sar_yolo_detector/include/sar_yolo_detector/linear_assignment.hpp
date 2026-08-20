#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sar_yolo_detector {

// Rectangular Hungarian assignment. Rows are observations and columns are
// existing tracks. A row receives -1 when every real assignment is at least
// unmatched_cost. Dummy columns make creation of a new track explicit.
inline std::vector<int>
minimumCostAssignment(const std::vector<std::vector<double>> &real_costs,
                      const double unmatched_cost) {
  const std::size_t row_count = real_costs.size();
  if (row_count == 0)
    return {};
  const std::size_t real_column_count = real_costs.front().size();
  const std::size_t column_count = real_column_count + row_count;
  const double blocked_cost = std::max(1.0e6, unmatched_cost * 1.0e6);

  std::vector<double> u(row_count + 1, 0.0), v(column_count + 1, 0.0);
  std::vector<int> p(column_count + 1, 0), way(column_count + 1, 0);
  for (std::size_t row = 1; row <= row_count; ++row) {
    p[0] = static_cast<int>(row);
    int column0 = 0;
    std::vector<double> minimum(column_count + 1,
                                std::numeric_limits<double>::infinity());
    std::vector<bool> used(column_count + 1, false);
    do {
      used[static_cast<std::size_t>(column0)] = true;
      const int row0 = p[static_cast<std::size_t>(column0)];
      double delta = std::numeric_limits<double>::infinity();
      int column1 = 0;
      for (std::size_t column = 1; column <= column_count; ++column) {
        if (used[column])
          continue;
        double cost = unmatched_cost;
        if (column <= real_column_count) {
          cost = real_costs[static_cast<std::size_t>(row0 - 1)][column - 1];
          if (!std::isfinite(cost))
            cost = blocked_cost;
        } else if (column - real_column_count !=
                   static_cast<std::size_t>(row0)) {
          cost = blocked_cost;
        }
        const double current = cost - u[static_cast<std::size_t>(row0)] -
                               v[column];
        if (current < minimum[column]) {
          minimum[column] = current;
          way[column] = column0;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          column1 = static_cast<int>(column);
        }
      }
      for (std::size_t column = 0; column <= column_count; ++column) {
        if (used[column]) {
          u[static_cast<std::size_t>(p[column])] += delta;
          v[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      column0 = column1;
    } while (p[static_cast<std::size_t>(column0)] != 0);
    do {
      const int column1 = way[static_cast<std::size_t>(column0)];
      p[static_cast<std::size_t>(column0)] =
          p[static_cast<std::size_t>(column1)];
      column0 = column1;
    } while (column0 != 0);
  }

  std::vector<int> assignment(row_count, -1);
  for (std::size_t column = 1; column <= column_count; ++column) {
    if (p[column] <= 0)
      continue;
    const std::size_t row = static_cast<std::size_t>(p[column] - 1);
    if (column <= real_column_count &&
        std::isfinite(real_costs[row][column - 1]) &&
        real_costs[row][column - 1] < unmatched_cost) {
      assignment[row] = static_cast<int>(column - 1);
    }
  }
  return assignment;
}

} // namespace sar_yolo_detector
