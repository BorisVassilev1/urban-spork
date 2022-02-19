#pragma once

#include "Primitive.h"
#include "Utils.hpp"


struct TriangleMesh : Primitive {
	struct Triangle : Intersectable {
		int indices[3];
		TriangleMesh *owner = nullptr;

		Triangle(int v1, int v2, int v3, TriangleMesh *owner)
			: indices{v1, v2, v3}, owner(owner)
		{}

		bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override;
		bool boxIntersect(const BBox &box) override;
		void expandBox(BBox& box) override;
		vec3 getCenter() override; 
	};
	AcceleratorPtr accelerator;
	std::vector<vec3> vertices;
	std::vector<Triangle> faces;
	std::unique_ptr<Material> material;

	TriangleMesh(const std::string &objFile, std::unique_ptr<Material> material)
		: material(std::move(material)) {
		loadFromObj(objFile);
	}

	void onBeforeRender() override;
	bool loadFromObj(const std::string &objPath);

	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override;
	bool intersectTriangle(const Ray& ray, const Triangle &t, Intersection &info);
};

