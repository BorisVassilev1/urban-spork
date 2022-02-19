#include "Primitive.h"

SpherePrim::SpherePrim(vec3 center, float radius, MaterialPtr material): center(center), radius(radius), material(std::move(material)) {
	box.add(center);
	box.add(center + vec3(radius, radius, radius));
	box.add(center - vec3(radius, radius, radius));
}

bool SpherePrim::intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) {
	const float a = dot(ray.dir, ray.dir);
	const float b = 2.f * dot(ray.dir, ray.origin - center);
	const float c = dot(ray.origin - center, ray.origin - center) - radius * radius;
	const float D = b * b - 4 * a * c;
	if (D >= 0.f) {
		const float t = (-b - sqrtf(D)) / (2.f * a);
		if (t >= tMin && t <= tMax) {
			intersection.t = t;
			intersection.p = ray.at(t);
			intersection.normal = (intersection.p - center) / radius;
			intersection.material = material.get();
			return true;
		}
	}
	return false;
}

bool Instancer::Instance::intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) {
	const Ray local = {
		(ray.origin - offset) / scale,
		ray.dir
	};
	if (primitive->intersect(local, tMin, tMax, intersection)) {
		if (material) {
			intersection.material = material.get();
		}
		return true;
	}
	return false;
}

bool Instancer::Instance::boxIntersect(const BBox &other) {
	const BBox transformed {
		primitive->box.min * scale + offset,
		primitive->box.max * scale + offset
	};
	return !other.boxIntersection(transformed).isEmpty();
}

void Instancer::Instance::expandBox(BBox &other) {
	const BBox transformed {
		primitive->box.min * scale + offset,
		primitive->box.max * scale + offset
	};
	other.add(transformed);
}

vec3 Instancer::Instance::getCenter() {
	const BBox transformed {
		primitive->box.min * scale + offset,
		primitive->box.max * scale + offset
	};
	return transformed.center();	
}

void Instancer::onBeforeRender() {
	for (int c = 0; c < instances.size(); c++) {
		instances[c].primitive->onBeforeRender();
	}
	if (instances.size() < 50) {
		return;
	}

	if (!accelerator) {
		accelerator = makeDefaultAccelerator();
	}
	if (!accelerator->isBuilt()) {
		accelerator->clear();
		for (int c = 0; c < instances.size(); c++) {
			accelerator->addPrimitive(&instances[c]);
		}
		accelerator->build(IntersectionAccelerator::Purpose::Instances);
	}
}

void Instancer::addInstance(SharedPrimPtr prim, const vec3& offset, float scale, SharedMaterialPtr material) {
	BBox primBox;
	primBox.min = (prim->box.min * scale) + offset;
	primBox.max = (prim->box.max * scale) + offset;
	box.add(primBox);
	Instance instance;
	instance.primitive = std::move(prim);
	instance.offset = offset;
	instance.scale = scale;
	instance.material = material;
	instances.push_back(instance);
}

bool Instancer::intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) {
	if (!box.testIntersect(ray)) {
		return false;
	}
	if (accelerator && accelerator->isBuilt()) {
		return accelerator->intersect(ray, tMin, tMax, intersection);
	}
	float closest = tMax;
	bool hasHit = false;
	for (int c = 0; c < instances.size(); c++) {
		Intersection data;
		if (instances[c].intersect(ray, tMin, tMax, data)) {
			if (data.t < closest) {
				intersection = data;
				closest = data.t;
				hasHit = true;
			}
		}
	}
	return hasHit;
}
