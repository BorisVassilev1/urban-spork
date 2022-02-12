#pragma once

#include <cmath>
#include <ostream>
#include <random>
#include <cassert>
#include <cfloat>

static const int MAX_RAY_DEPTH = 35;
const float PI = 3.14159265358979323846;

inline float degToRad(float deg) {
	return (deg * PI) / 180.f;
}

inline bool similar(float a, float b) {
	return fabs(a - b) < 1e-4;
}

/// Basic vector with 3 floats
struct vec3 {
	union {
		struct {
			float x, y, z;
		};
		float _v[3];
	};

	vec3() {}

	vec3(float v)
		: x(v) , y(v) , z(v)
	{}

	vec3(float r, float g, float b)
		: x(r), y(g), z(b)
	{}

	float &operator[](int index) {
		assert(index < 3 && index >= 0);
		return _v[index];
	}

	const float &operator[](int index) const {
		assert(index < 3 && index >= 0);
		return _v[index];
	}

	vec3 operator-() const {
		return vec3(-x, -y, -z);
	}

	float length() const {
		return std::sqrt(lengthSquare());
	}

	float lengthSquare() const {
		return x * x + y * y + z * z;
	}

	vec3 &operator+=(const vec3 &other) {
		for (int c = 0; c < 3; c++) {
			_v[c] += other._v[c];
		}
		return *this;
	}

	vec3 &operator*=(float v) {
		for (int c = 0; c < 3; c++) {
			_v[c] *= v;
		}
		return *this;
	}

	vec3 &operator/=(float v) {
		for (int c = 0; c < 3; c++) {
			_v[c] /= v;
		}
		return *this;
	}

	vec3 normalized() const {
		vec3 copy(*this);
		return copy /= length();
	}

	void normalize() {
		*this /= length();
	}

	bool similar(const vec3 &other) const {
		return ::similar(x, other.x) &&
			::similar(y, other.y) &&
			::similar(z, other.z);
	}

	bool isNormal() const {
		return similar(normalized());
	}

	bool operator>(const vec3 &other) const {
		return x > other.x && y > other.y && z > other.z;
	}

	vec3 inverted() const {
		return vec3{1.f / x, 1.f / y, 1.f / z};
	}
};

inline std::ostream &operator<<(std::ostream &out, const vec3 &v) {
	return out << v._v[0] << ' ' << v._v[1] << ' ' << v._v[2];
}

inline vec3 operator+(const vec3 &u, const vec3 &v) {
	return vec3(u._v[0] + v._v[0], u._v[1] + v._v[1], u._v[2] + v._v[2]);
}

inline vec3 operator-(const vec3 &u, const vec3 &v) {
	return vec3(u._v[0] - v._v[0], u._v[1] - v._v[1], u._v[2] - v._v[2]);
}

inline vec3 operator*(const vec3 &u, const vec3 &v) {
	return vec3(u._v[0] * v._v[0], u._v[1] * v._v[1], u._v[2] * v._v[2]);
}

inline vec3 operator*(float t, const vec3 &v) {
	return vec3(t * v._v[0], t * v._v[1], t * v._v[2]);
}

inline vec3 operator*(const vec3 &v, float t) {
	return t * v;
}

inline vec3 operator/(vec3 v, float t) {
	return (1 / t) * v;
}

inline vec3 max(const vec3 &a, const vec3 &b) {
	return {
		std::max(a.x, b.x),
		std::max(a.y, b.y),
		std::max(a.z, b.z),
	};
}

inline vec3 min(const vec3 &a, const vec3 &b) {
	return {
		std::min(a.x, b.x),
		std::min(a.y, b.y),
		std::min(a.z, b.z),
	};
}

inline float dot(const vec3 &u, const vec3 &v) {
	return u._v[0] * v._v[0]
		+ u._v[1] * v._v[1]
		+ u._v[2] * v._v[2];
}

inline vec3 cross(const vec3 &u, const vec3 &v) {
	return vec3(u._v[1] * v._v[2] - u._v[2] * v._v[1],
		u._v[2] * v._v[0] - u._v[0] * v._v[2],
		u._v[0] * v._v[1] - u._v[1] * v._v[0]);
}

/// Ray represented by origin and direction
struct Ray {
	vec3 origin;
	vec3 dir;

	Ray() {}

	Ray(const vec3 &origin, const vec3 &dir)
		: origin(origin), dir(dir) {
		assert(dir.isNormal());
	}

	vec3 at(float t) const {
		return origin + dir * t;
	}
};

/// @brief Get random float in range [0, 1]
inline float randFloat() {
	thread_local std::mt19937 rng(42);
	std::uniform_real_distribution<float> dist(0.f, 1.f);
	return dist(rng);
}

inline vec3 randomUnitSphere() {
	vec3 p;
	do {
		p = 2.0f * vec3(randFloat(), randFloat(), randFloat()) - vec3(1);
	} while (p.lengthSquare() >= 1.f);
	return p;
}

/// @brief Reflect vector @v from a surface with a normal @normal
/// @param v - the vector to reflect
/// @param normal - the surface normal
/// @return the reflected vector
inline vec3 reflect(const vec3 &v, const vec3 &normal) {
	return v - 2.f * dot(v, normal) * normal;
}

/// Axis aligned bounding box, needs only min and max point of the box
struct BBox {
	vec3 min = {FLT_MAX, FLT_MAX, FLT_MAX};
	vec3 max = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

	BBox() = default;

	/// @brief Check if the box is "empty", meaning its size has at lease one negative component
	bool isEmpty() const {
		const vec3 size = max - min;
		return size.x <= 1e-6f ||
			size.y <= 1e-6f ||
			size.z <= 1e-6f;
	}

	/// @brief Expand the box with another box, this will be union of both
	void add(const BBox &other) {
		min = ::min(min, other.min);
		max = ::max(max, other.max);
	}

	/// @brief Expand the box with a single point
	void add(const vec3 &point) {
		min = ::min(min, point);
		max = ::max(max, point);
	}

	/// @brief Check if given point is inside the box with some tolerance
	bool inside(const vec3& point) const {
		return (min.x - 1e-6 <= point.x && point.x <= max.x + 1e-6 &&
		        min.y - 1e-6 <= point.y && point.y <= max.y + 1e-6 &&
		        min.z - 1e-6 <= point.z && point.z <= max.z + 1e-6);
	}

	/// @brief Split the box in 8 equal parts, children are not sorted in any way
	/// @param parts [out] - where to write the children
	void octSplit(BBox parts[8]) const {
		assert(!isEmpty());
		const vec3 size = max - min;
		const vec3 center = min + size / 2;

		parts[0] = BBox{min, center};
		parts[1] = BBox{center, max};

		parts[2] = BBox{vec3{min.x, center.y, min.z}, vec3{center.x, max.y, center.z}};
		parts[3] = BBox{vec3{min.x, center.y, center.z}, vec3{center.x, max.y, max.z}};
		parts[4] = BBox{vec3{min.x, min.y, center.z}, vec3{center.x, center.y, max.z}};

		parts[5] = BBox{vec3{center.x, min.y, min.z}, vec3{max.x, center.y, center.z}};
		parts[6] = BBox{vec3{center.x, min.y, center.z}, vec3{max.x, center.y, max.z}};
		parts[7] = BBox{vec3{center.x, center.y, min.z}, vec3{max.x, max.y, center.z}};
	}

	/// @brief Compute the intersection with another box
	///	@return empty box if there is no intersection
	BBox boxIntersection(const BBox &other) const {
		assert(!isEmpty());
		assert(!other.isEmpty());
		return {
			::max(min, other.min),
			::min(max, other.max)
		};
	}

	/// @brief Check if a ray intersects the box
	bool testIntersect(const Ray& ray) const {
		// source: https://github.com/anrieff/quaddamage/blob/master/src/bbox.h
		assert(!isEmpty());
		if (inside(ray.origin)) {
			return true;
		}

		for (int dim = 0; dim < 3; dim++) {
			if ((ray.dir[dim] < 0 && ray.origin[dim] < min[dim]) || (ray.dir[dim] > 0 && ray.origin[dim] > max[dim])) {
				continue;
			}
			if (fabs(ray.dir[dim]) < 1e-9) {
				continue;
			}
			const float mul = 1.f / ray.dir[dim];
			const int u = (dim == 0) ? 1 : 0;
			const int v = (dim == 2) ? 1 : 2;
			float dist = (min[dim] - ray.origin[dim]) * mul;
			if (dist < 0) {
				continue; //*
			}
			/* (*) this is a good optimization I found out by chance. Consider the following scenario
			 *
			 *   ---+  ^  (ray)
			 *      |   \
			 * bbox |    \
			 *      |     \
			 *      |      * (camera)
			 * -----+
			 *
			 * if we're considering the walls up and down of the bbox (which belong to the same axis),
			 * the optimization in (*) says that we can skip testing with the "up" wall, if the "down"
			 * wall is behind us. The rationale for that is, that we can never intersect the "down" wall,
			 * and even if we have the chance to intersect the "up" wall, we'd be intersection the "right"
			 * wall first. So we can just skip any further intersection tests for this axis.
			 * This may seem bogus at first, as it doesn't work if the camera is inside the BBox, but then we would
			 * have quitted the function because of the inside(ray.start) condition in the first line of the function.
			 */
			float x = ray.origin[u] + ray.dir[u] * dist;
			if (min[u] <= x && x <= max[u]) {
				const float y = ray.origin[v] + ray.dir[v] * dist;
				if (min[v] <= y && y <= max[v]) {
					return true;
				}
			}
			dist = (max[dim] - ray.origin[dim]) * mul;
			if (dist < 0) {
				continue;
			}
			x = ray.origin[u] + ray.dir[u] * dist;
			if (min[u] <= x && x <= max[u]) {
				const float y = ray.origin[v] + ray.dir[v] * dist;
				if (min[v] <= y && y <= max[v]) {
					return true;
				}
			}
		}
		return false;
	}
};
