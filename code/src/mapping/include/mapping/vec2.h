#ifndef VEC2_H
#define VEC2_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>
#include <type_traits>

template<class T>
class Vec2 {
    public:
        Vec2()
            : x(0), y(0) {}

        Vec2(T x, T y)
            : x(x), y(y) {}

        Vec2(geometry_msgs::msg::Point p) requires std::is_floating_point_v<T>
            : x(p.x), y(p.y) {}

        Vec2(geometry_msgs::msg::PointStamped p) requires std::is_floating_point_v<T>
            : Vec2<T>(p.point) {}

        [[nodiscard]] geometry_msgs::msg::Point toGeometryMsgPoint() const requires std::is_floating_point_v<T> {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            return p;
        }

        [[nodiscard]] geometry_msgs::msg::PointStamped toGeometryMsgPointStamped(std_msgs::msg::Header header) const requires std::is_floating_point_v<T> {
            geometry_msgs::msg::PointStamped p;
            p.header = header;
            p.point = toGeometryMsgPoint();
            return p;
        }

        [[nodiscard]] static Vec2<T> fromAngle(T angle) requires std::is_floating_point_v<T> {
            return Vec2<T>(std::cos(angle), std::sin(angle));
        }

        Vec2<T> operator+(const Vec2<T> b) const {
            return Vec2(x + b.x, y + b.y);
        }

        Vec2<T> operator-(Vec2<T> b) const {
            return Vec2<T>(x - b.x, y - b.y);
        }

        Vec2<T> operator*(T s) const {
            return Vec2<T>(x * s, y * s);
        }

        friend Vec2<T> operator*(T s, Vec2<T> b) {
            return b * s;
        }

        Vec2<T> operator/(T s) const {
            return Vec2<T>(x / s, y / s);
        }

        Vec2<T> &operator+=(Vec2<T> b) {
            *this = *this + b;
            return *this;
        }

        Vec2<T> &operator-=(Vec2<T> b) {
            *this = *this - b;
            return *this;
        }

        [[nodiscard]] T norm() const requires std::is_floating_point_v<T> {
            return std::sqrt(x * x + y * y);
        }

        [[nodiscard]] T angle() const requires std::is_floating_point_v<T> {
            return std::atan2(y, x);
        }

        friend T crossProduct(Vec2<T> a, Vec2<T> b) {
            return a.x * b.y - a.y * b.x;
        }

        /*[[nodiscard]] T relativeAngleTo(Vec2<T> b) const requires std::is_floating_point_v<T> {
            return std::acos(dotProduct(this->normalized(), b.normalized()));
        }*/

        [[nodiscard]] Vec2<T> normalized() const requires std::is_floating_point_v<T> {
            return *this / this->norm();
        }

        [[nodiscard]] Vec2<T> rotated(T angle) const requires std::is_floating_point_v<T> {
            return Vec2<T>(x * std::cos(angle) - y * std::sin(angle),
                           x * std::sin(angle) + y * std::cos(angle));
        }

        /*Vec2<T> &normalize() requires std::is_floating_point_v<T> {
            *this = normalized();
            return *this;
        }

        Vec2<T> &rotate(T angle) requires std::is_floating_point_v<T> {
            *this = rotated(angle);
            return *this;
        }*/

        friend bool operator<(Vec2<T> a, Vec2<T> b) {
            if (a.x != b.x) {
                return a.x < b.x;
            } else {
                return a.y < b.y;
            }
        }

        friend bool operator==(Vec2<T> a, Vec2<T> b) {
            return a.x == b.x && a.y == b.y;
        }

        friend bool operator!=(Vec2<T> a, Vec2<T> b) {
            return a.x != b.x || a.y != b.y;
        }

        T x;
        T y;
};

using Vec2f = Vec2<double>;
using Vec2i = Vec2<int>;

#endif
