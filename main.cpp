#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <float.h>
#include <vector>


struct Point {
    double xx;
    double yy;
    Point(double xx_ = 0.0, double yy_ = 0.0): xx(xx_), yy(yy_) {}
};


struct IntersectionPoint {
    int side;
    Point coord;
    IntersectionPoint(int side_ = 0, Point coord_ = Point()): side(side_), coord(coord_) {}
};


struct Intersection {
    IntersectionPoint left;
    IntersectionPoint right;
    Intersection(IntersectionPoint left_ = IntersectionPoint(), IntersectionPoint right_ = IntersectionPoint()):
        left(left_), right(right_) {}

};


struct LineCoefficients {
    double A;
    double B;
    double C;
    LineCoefficients(double A_ = 0.0, double B_ = 0.0, double C_ = 0.0): A(A_), B(B_), C(C_) {}
    //Ax + By + C = 0
};


struct Segment {
    Point head;
    Point tail;
    Segment(Point head_ = Point(), Point tail_ = Point()): head(head_), tail(tail_) {}
};

struct OffsetRange {
    double left;
    double right;
    OffsetRange(double left_ = 0.0, double right_ = 0.0): left(left_), right(right_) {}
    double middle_of_range()
    {
        return (left + right) / 2.0;
    }
};

struct PolygonPartition {
    double angle;
    Point center;
    double area_difference;
    PolygonPartition(double angle_ = 0.0, Point center_ = Point(), double area_difference_ = 0.0):
        angle(angle_), center(center_), area_difference(area_difference_) {}
};

struct PolygonPartitionRange {
    PolygonPartition min;
    PolygonPartition max;
    PolygonPartitionRange(PolygonPartition min_angle_polygon_partition_ = PolygonPartition(),
                          PolygonPartition max_angle_polygon_partition_ = PolygonPartition()):
        min(min_angle_polygon_partition_),
        max(max_angle_polygon_partition_) {}
};

const double EPS = 1e-7;

double polygon_area(const std::vector<Point>& polygon);
Intersection find_bisection(const Point& line_direction, const std::vector<Point>& polygon);
PolygonPartition find_polygon_partition_for_angle(const std::vector<Point>& polygon, double angle);
bool find_line_polygon_side_intersection(const LineCoefficients line_coeff, const Segment& side_segment,
                                         int side, IntersectionPoint& inter);
Intersection find_line_polygon_intersection(const LineCoefficients& cur_coeff, const std::vector<Point>& polygon);
LineCoefficients find_bisection_line_coefficients(const Intersection& inter);
Point find_bisections_intersection_point(const LineCoefficients& bisection1,
                                         const LineCoefficients& bisection2);
double find_distance_to_side_beginning(const Intersection& inter,
                                       const Point& side_beginning);
IntersectionPoint find_polygon_breakpoint(const Intersection& inter1,
                                          const Intersection& inter2,
                                          const std::vector<Point>& polygon);
double polygon_area_part(const Point& left, const Point& right);
void register_line_polygon_side_intersection(const Segment& side_segment, const LineCoefficients& cur_coeff,
                                             const int side, std::vector<IntersectionPoint>& intersections);
std::vector<Point> construct_polygon_part(const std::vector<Point>& polygon,
                                          const IntersectionPoint &left_edge_point, const IntersectionPoint &right_edge_point);
std::vector<Point> construct_polygon_part(const std::vector<Point>& polygon,
                                          const IntersectionPoint &left_edge_point, const IntersectionPoint &right_edge_point,
                                          const Point& center);
bool are_equal(double val1, double val2, double eps);
bool is_half_consisting_min_offset_ind_greater(const int min_offset_ind, const double find_polygon_partition_for_angle, const Intersection& inter);
bool is_first_intersection_side_greater_than_second(const Intersection& inter1,
                                                    const Intersection& inter2);
bool is_first_intersection_to_right_of_second_on_one_side(const Intersection& inter1,
                                                          const Intersection& inter2,
                                                          const std::vector<Point>& polygon);
double find_distance_between_points(const Point& point1, const Point& point2);
PolygonPartitionRange find_optimal_polygon_partition(const std::vector<Point>& polygon, const double eps);
double get_degree_angle(double radian_angle);
void find_offset_range(const Point& line_direction, const std::vector<Point>& polygon,
                       int& min_offset_ind);
Point find_intersection_center(Intersection inter1, Intersection inter2);
std::vector<Point> get_polygon();
std::ostream& operator<<(std::ostream& stream, const Point& point);
std::ostream& operator<<(std::ostream& stream, const PolygonPartition& polygon_partition);
PolygonPartitionRange get_initial_polygon_partition_range(const std::vector<Point>& polygon);
bool is_polygon_partition_range_narrow_with_precision(PolygonPartitionRange polygon_partition_range,
                                                      const double eps);
PolygonPartition get_min_angle_polygon_partition(const std::vector<Point>& polygon);
PolygonPartition get_max_angle_polygon_partition(const std::vector<Point>& polygon);
std::ostream& operator<<(std::ostream& stream, const PolygonPartitionRange& polygon_partition_range);



int main()
{
    std::vector<Point> polygon = get_polygon();
    PolygonPartitionRange optimal_polygon_partition = find_optimal_polygon_partition(polygon, EPS);
    std::cout << optimal_polygon_partition;
    return 0;
}

std::ostream& operator<<(std::ostream& stream, const Point& point)
{
    return stream << std::setprecision(7) << point.xx << " " << point.yy << std::endl;
}

std::ostream& operator<<(std::ostream& stream, const PolygonPartition& polygon_partition)
{
    return stream << polygon_partition.center << std::setprecision(7) <<
                     get_degree_angle(polygon_partition.angle) << std::endl;
}

std::ostream& operator<<(std::ostream& stream, const PolygonPartitionRange& polygon_partition_range)
{
    return stream << polygon_partition_range.min;
}

std::vector<Point> get_polygon()
{
    std::vector<Point> polygon;
    polygon.push_back(Point(0, 0));
    polygon.push_back(Point(5, 0));
    polygon.push_back(Point(0, 5));
    return polygon;
}

double get_degree_angle(double radian_angle)
{
    return (radian_angle * 180) / M_PI;
}

bool is_polygon_partition_range_narrow_with_precision(PolygonPartitionRange polygon_partition_range,
                                                      const double eps)
{
    return !are_equal(polygon_partition_range.min.angle,
                      polygon_partition_range.max.angle, eps);
}

PolygonPartition get_min_angle_polygon_partition(const std::vector<Point>& polygon)
{
    return find_polygon_partition_for_angle(polygon, 0);
}

PolygonPartition get_max_angle_polygon_partition(const std::vector<Point>& polygon)
{
    return find_polygon_partition_for_angle(polygon, M_PI_2);
}

PolygonPartitionRange get_initial_polygon_partition_range(const std::vector<Point>& polygon)
{
    return PolygonPartitionRange(get_min_angle_polygon_partition(polygon),
                                 get_max_angle_polygon_partition(polygon));
}

PolygonPartitionRange find_optimal_polygon_partition(const std::vector<Point>& polygon, const double eps)
{
    PolygonPartitionRange polygon_partition_range = get_initial_polygon_partition_range(polygon);
    PolygonPartition cur_angle_polygon_partition;
    while (is_polygon_partition_range_narrow_with_precision(polygon_partition_range, eps)) {
        cur_angle_polygon_partition =
                find_polygon_partition_for_angle(polygon, (polygon_partition_range.min.angle + polygon_partition_range.max.angle) / 2.0);
        if (polygon_partition_range.min.area_difference * cur_angle_polygon_partition.area_difference < 0.0) {
            polygon_partition_range.max = cur_angle_polygon_partition;
        } else {
            polygon_partition_range.min = cur_angle_polygon_partition;
        }
    }
    return polygon_partition_range;
}

double polygon_area(const std::vector<Point>& polygon)
{
    double polygon_area = 0;
    for (size_t i = 0; i < polygon.size() - 1; ++i) {
        polygon_area += polygon_area_part(polygon[i], polygon[i + 1]);
    }
    polygon_area += polygon_area_part(polygon.back(), polygon.front());
    polygon_area = fabs(polygon_area) / 2.0;
    return polygon_area;
}


double polygon_area_part(const Point& left, const Point& right)
{
    return (right.xx - left.xx) * (right.yy + left.yy);
}


void find_offset_range(const Point& line_direction, const std::vector<Point>& polygon,
                       int& min_offset_ind, OffsetRange& offset_range)
{
    min_offset_ind = 0;
    offset_range.left = line_direction.xx * polygon[min_offset_ind].xx + line_direction.yy * polygon[min_offset_ind].yy;
    offset_range.right = offset_range.left;
    for (size_t i = 0; i < polygon.size(); ++i) {
        double cur_offset = line_direction.xx * polygon[i].xx + line_direction.yy * polygon[i].yy;
        if (cur_offset > offset_range.right) {
            offset_range.right = cur_offset;
        } else if (cur_offset < offset_range.left) {
            min_offset_ind = i;
            offset_range.left = cur_offset;
        }
    }
}



Intersection find_line_polygon_intersection(const Point& line_direction, double offset,
                                            const std::vector<Point>& polygon)
{
    LineCoefficients cur_coeff(line_direction.xx, line_direction.yy, offset);
    return find_line_polygon_intersection(cur_coeff, polygon);
}

Intersection find_bisection(const Point& line_direction, const std::vector<Point>& polygon)
{
    int min_offset_ind;
    OffsetRange offset_range;
    find_offset_range(line_direction, polygon, min_offset_ind, offset_range);

    const double whole_area = polygon_area(polygon);
    double area_difference = whole_area;
    Intersection inter;

    while (!are_equal(area_difference, 0.0, EPS * whole_area)) {
        double cur_offset = offset_range.middle_of_range();
        inter = find_line_polygon_intersection(line_direction, cur_offset,  polygon);
        std::vector<Point> half_pol = construct_polygon_part(polygon, inter.left, inter.right);

        area_difference = whole_area - 2 * polygon_area(half_pol);

        (is_half_consisting_min_offset_ind_greater(min_offset_ind, area_difference, inter) ? offset_range.right : offset_range.left) = cur_offset;
    }
    return inter;
}

bool is_half_consisting_min_offset_ind_greater(const int min_offset_ind, const double area_difference, const Intersection& inter)
{
    bool is_min_offset_ind_between_intersection_sides = (min_offset_ind > inter.left.side && min_offset_ind <= inter.right.side);
    return ((is_min_offset_ind_between_intersection_sides && area_difference < 0)||
            (!is_min_offset_ind_between_intersection_sides && area_difference >= 0));
}

LineCoefficients find_bisection_line_coefficients(const Intersection& inter)
{
    return LineCoefficients(inter.left.coord.yy - inter.right.coord.yy,
                            inter.right.coord.xx - inter.left.coord.xx,
                            inter.left.coord.xx * inter.right.coord.yy -
                            inter.right.coord.xx * inter.left.coord.yy);
}


Point find_bisections_intersection_point(const LineCoefficients& bisection1,
                                         const LineCoefficients& bisection2)
{
    double down = bisection2.A * bisection1.B - bisection1.A * bisection2.B;
    return Point((bisection1.C * bisection2.B - bisection2.C * bisection1.B) / down,
                 (bisection1.A * bisection2.C - bisection2.A * bisection1.C) / down);
}


double find_distance_to_side_beginning(const Intersection& inter,
                                       const Point& side_beginning)
{
    return find_distance_between_points(inter.left.coord, side_beginning);
}

double find_distance_between_points(const Point& point1, const Point& point2)
{
    return (point1.xx - point2.xx) * (point1.xx - point2.xx) +
            (point1.yy - point2.yy) * (point1.yy - point2.yy);
}

IntersectionPoint find_polygon_breakpoint(const Intersection& inter1,
                                          const Intersection& inter2,
                                          const std::vector<Point>& polygon)
{
    if (is_first_intersection_side_greater_than_second(inter1, inter2) ||
            is_first_intersection_to_right_of_second_on_one_side(inter1, inter2, polygon)) {
        return inter2.right;
    } else {
        return inter2.left;
    }
}

bool is_first_intersection_side_greater_than_second(const Intersection& inter1,
                                                    const Intersection& inter2)
{
    return (inter1.left.side > inter2.left.side);
}

bool is_first_intersection_to_right_of_second_on_one_side(const Intersection& inter1,
                                                          const Intersection& inter2,
                                                          const std::vector<Point>& polygon)
{
    return (inter1.left.side == inter2.left.side &&
            find_distance_to_side_beginning(inter1, polygon[inter1.left.side]) >
            find_distance_to_side_beginning(inter2, polygon[inter2.left.side]));
}

Point find_intersection_center(Intersection inter1, Intersection inter2)
{
    LineCoefficients line_fir = find_bisection_line_coefficients(inter1);
    LineCoefficients line_sec = find_bisection_line_coefficients(inter2);
    return find_bisections_intersection_point(line_fir, line_sec);
}

PolygonPartition find_polygon_partition_for_angle(const std::vector<Point>& polygon, double angle)
{
    Intersection inter1 = find_bisection(Point(sin(angle), -cos(angle)), polygon);
    Intersection inter2 = find_bisection(Point(cos(angle), sin(angle)), polygon);

    Point center = find_intersection_center(inter1, inter2);


    IntersectionPoint stop = find_polygon_breakpoint(inter1, inter2, polygon);


    std::vector<Point> quarter_pol = construct_polygon_part(polygon, inter1.left, stop, center);

    double quarter_area = polygon_area(quarter_pol);

    return PolygonPartition(angle, center, 4 * quarter_area - polygon_area(polygon));
}


bool find_line_polygon_side_intersection(const LineCoefficients line_coeff, const Segment& side_segment,
                                         int side, IntersectionPoint& inter)
{
    const Point line_direction(line_coeff.A, line_coeff.B);
    const double cur_d = line_coeff.C;
    bool is_intersected = false;
    double test_cur = line_direction.xx * side_segment.head.xx + line_direction.yy * side_segment.head.yy - cur_d;
    double test_next = line_direction.xx * side_segment.tail.xx + line_direction.yy * side_segment.tail.yy - cur_d;

    if (test_cur * test_next < 0.0) {
        is_intersected = true;
        inter.side = side;
        LineCoefficients side_line(side_segment.head.yy - side_segment.tail.yy,
                                   side_segment.tail.xx - side_segment.head.xx,
                                   side_segment.head.xx * side_segment.tail.yy -
                                   side_segment.tail.xx * side_segment.head.yy);
        double down = line_direction.xx * side_line.B - side_line.A * line_direction.yy;
        inter.coord = Point((cur_d * side_line.B + side_line.C * line_direction.yy) / down,
                            (-line_direction.xx * side_line.C - side_line.A * cur_d) / down);
    } else if (are_equal(test_cur, 0.0, DBL_EPSILON) && !are_equal(test_next, 0.0, DBL_EPSILON)) {
        is_intersected = true;
        inter.side = side;
        inter.coord = side_segment.head;
    }

    return is_intersected;
}

bool are_equal(double val1, double val2, double eps)
{
    return (fabs(val1 - val2) < eps);
}

Intersection find_line_polygon_intersection(const LineCoefficients& cur_coeff, const std::vector<Point>& polygon)
{
    std::vector<IntersectionPoint> ans_intersections;
    for (size_t side = 0; side < polygon.size() - 1; ++side) {
        register_line_polygon_side_intersection(Segment(polygon[side], polygon[side + 1]), cur_coeff,
                side, ans_intersections);
    }
    register_line_polygon_side_intersection(Segment(polygon.back(), polygon.front()), cur_coeff,
                                            polygon.size() - 1, ans_intersections);

    return Intersection(ans_intersections[0], ans_intersections[1]);
}


void register_line_polygon_side_intersection(const Segment& side_segment, const LineCoefficients& cur_coeff,
                                             const int side, std::vector<IntersectionPoint>& intersections)
{
    IntersectionPoint cur_inter;
    if (find_line_polygon_side_intersection(cur_coeff, side_segment, side, cur_inter)) {
        intersections.push_back(cur_inter);
    }
}


std::vector<Point> construct_polygon_part(const std::vector<Point>& polygon, const IntersectionPoint& left_edge_point,
                                          const IntersectionPoint& right_edge_point)
{
    const int left_edge = left_edge_point.side + 1;
    const int right_edge = right_edge_point.side + 1;
    std::vector<Point> part_pol(1, left_edge_point.coord);
    part_pol.insert(part_pol.begin() + 1, polygon.begin() + left_edge, polygon.begin() + right_edge);
    part_pol.push_back(right_edge_point.coord);
    return part_pol;
}

std::vector<Point> construct_polygon_part(const std::vector<Point>& polygon,
                                          const IntersectionPoint& left_edge_point, const IntersectionPoint& right_edge_point,
                                          const Point& center)
{
    std::vector<Point> part_pol = construct_polygon_part(polygon, left_edge_point, right_edge_point);
    part_pol.push_back(center);
    return part_pol;

}
