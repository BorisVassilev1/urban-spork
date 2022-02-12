#pragma once
#include "Image.hpp"
#include "Utils.hpp"
#include <memory>

struct Intersection;

/// Base class for a surface material
struct Material {
	/// @brief Shade an intersection. Called from multiple threads
	/// @param in - the ray that created the intersection
	/// @param data - surface properties of the intersection
	/// @param attenuation [out] - color attenuation for the at the intersection
	/// @param scatter [out] - new scatter ray from the intersection
	virtual bool shade(const Ray &in, const Intersection &data, Color &attenuation, Ray &scatter) = 0;
};

typedef std::unique_ptr<Material> MaterialPtr;
typedef std::shared_ptr<Material> SharedMaterialPtr;

struct Lambert : Material {
	Color albedo;
	Lambert(Color albedo)
		: albedo(albedo)
	{}
	bool shade(const Ray& ray, const Intersection& data, Color& attenuation, Ray& scatter) override;
};

struct Metal : Material {
	Color albedo;
	float fuzz;
	Metal(Color albedo, float fuzz)
		: albedo(albedo)
		, fuzz(fuzz)
	{}
	bool shade(const Ray& ray, const Intersection& data, Color& attenuation, Ray& scatter) override;
};
