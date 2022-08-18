//
// Created by lfc on 2020/2/26.
//

#ifndef SRC_BRESEN_POINTS_BUILDER_HPP
#define SRC_BRESEN_POINTS_BUILDER_HPP
namespace tsdf {

template <class Point>
class BresenPointsBuilder {
 public:
    static void computeBresenPoints(const Point &begin, const Point &end, std::vector<Point> &points,
                                    bool include_end = true) {
        int dx = end.x() - begin.x();
        int dy = end.y() - begin.y();
        int abs_dx = abs(dx);
        int abs_dy = abs(dy);
        int offset_dx = dx > 0 ? 1 : -1;
        int offset_dy = dy > 0 ? 1 : -1;
        points.clear();
        if (include_end) {
            points.push_back(begin);
        }
        if (abs_dx >= abs_dy) {
            int err_y = abs_dx >> 1;
            computePointsX(abs_dx, abs_dy, err_y, offset_dx, offset_dy, begin.x(), begin.y(), points);
        } else {
            int err_x = abs_dy >> 1;
            computePointsY(abs_dx, abs_dy, err_x, offset_dx, offset_dy, begin.x(), begin.y(), points);
        }
        if (include_end) {
            points.push_back(end);
        }
    }

 private:
    static void computePointsX(int abs_dx, int abs_dy, int error_y, int offset_dx, int offset_dy, int start_x,
                               int start_y, std::vector<Point> &points) {
        int end_x = start_x + abs_dx * (offset_dx);
        int end_y = start_y + abs_dy * (offset_dy);
        int end = abs_dx - 1;
        for (int i = 0; i < end; i++) {
            start_x += offset_dx;
            error_y += abs_dy;
            if (error_y >= abs_dx) {
                start_y += offset_dy;
                error_y -= abs_dx;
            }
            points.emplace_back();
            points.back().x() = start_x;
            points.back().y() = start_y;
        }
    }

    static inline void computePointsY(int abs_dx, int abs_dy, int error_x, int offset_dx, int offset_dy, int start_x,
                                      int start_y, std::vector<Point> &points) {
        int end_x = start_x + abs_dx * (offset_dx);
        int end_y = start_y + abs_dy * (offset_dy);
        int end = abs_dy - 1;
        for (int i = 0; i < end; i++) {
            start_y += offset_dy;
            error_x += abs_dx;
            if (error_x >= abs_dy) {
                start_x += offset_dx;
                error_x -= abs_dy;
            }
            points.emplace_back();
            points.back().x() = start_x;
            points.back().y() = start_y;
        }
    }
};

}  // namespace tsdf

#endif  // SRC_BRESEN_POINTS_BUILDER_HPP
