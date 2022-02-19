#define TINYOBJLOADER_IMPLEMENTATION

#include "Mesh.h"
#include "third_party/tiny_obj_loader.h"

/// source https://github.com/anrieff/quaddamage/blob/master/src/mesh.cpp
bool intersectTriangleFast(const Ray& ray, const vec3& A, const vec3& B, const vec3& C, float& dist)
{
	const vec3 AB = B - A;
	const vec3 AC = C - A;
	const vec3 D = -ray.dir;
	//              0               A
	const vec3 H = ray.origin - A;

	/* 2. Solve the equation:
	 *
	 * A + lambda2 * AB + lambda3 * AC = ray.start + gamma * ray.dir
	 *
	 * which can be rearranged as:
	 * lambda2 * AB + lambda3 * AC + gamma * D = ray.start - A
	 *
	 * Which is a linear system of three rows and three unknowns, which we solve using Carmer's rule
	 */

	// Find the determinant of the left part of the equation:
	const vec3 ABcrossAC = cross(AB, AC);
	const float Dcr = dot(ABcrossAC, D);

	// are the ray and triangle parallel?
	if (fabs(Dcr) < 1e-12) {
		return false;
	}

	const float lambda2 = dot( cross( H, AC), D ) / Dcr;
	const float lambda3 = dot( cross(AB, H), D ) / Dcr;
	const float gamma   = dot( ABcrossAC, H ) / Dcr;

	// is intersection behind us, or too far?
	if (gamma < 0 || gamma > dist) {
		return false;
	}

	// is the intersection outside the triangle?
	if (lambda2 < 0 || lambda2 > 1 || lambda3 < 0 || lambda3 > 1 || lambda2 + lambda3 > 1) {
		return false;
	}

	dist = gamma;

	return true;
}


/// source: https://github.com/anrieff/quaddamage/blob/master/src/bbox.h
bool TriangleMesh::Triangle::intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) {
	const vec3 &A = owner->vertices[indices[0]];
	const vec3 &B = owner->vertices[indices[1]];
	const vec3 &C = owner->vertices[indices[2]];

	const vec3 AB = B - A;
	const vec3 AC = C - A;

	const vec3 normal = cross(AB, AC).normalized();;

	if (dot(ray.dir, normal) > 0) {
		return false;
	}

	const vec3 ABcrossAC = cross(AB, AC);
	
	const vec3 H = ray.origin - A;
	const vec3 D = ray.dir;
	
	const float Dcr = - dot(ABcrossAC, D);

	if (fabs(Dcr) < 1e-12) {
		return false;
	}

	const float rDcr = 1.f / Dcr;
	const float gamma = dot(ABcrossAC, H) * rDcr;
	if (gamma < tMin || gamma > tMax) {
		return false;
	}

	const vec3 HcrossD = cross(H, D);
	const float lambda2 = dot(HcrossD, AC) * rDcr;
	if (lambda2 < 0 || lambda2 > 1) {
		return false;
	}

	const float lambda3 = -dot(AB, HcrossD) * rDcr;
	if (lambda3 < 0 || lambda3 > 1) {
		return false;
	}

	if (lambda2 + lambda3 > 1) {
		return false;
	}

	intersection.t = gamma;
	intersection.p = ray.origin + ray.dir * gamma;
	intersection.normal = normal;
	intersection.material = owner->material.get();

	return true;
}

int signOf(float f) {
	return (f > 0) - (f < 0);
}

bool TriangleMesh::Triangle::boxIntersect(const BBox& box) {
	const vec3 &A = owner->vertices[indices[0]];
	const vec3 &B = owner->vertices[indices[1]];
	const vec3 &C = owner->vertices[indices[2]];
	if (box.inside(A) || box.inside(B) || box.inside(C)) {
		return true;
	}

	Ray ray;
	vec3 t[3] = { A, B, C };
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 3; j++) {
			ray.origin = t[i];
			ray.dir = t[j] - t[i];
			if (box.testIntersect(ray)) {
				ray.origin = t[j];
				ray.dir = t[i] - t[j];
				if (box.testIntersect(ray)) {
					return true;
				}
			}
		}
	}
	vec3 AB = B - A;
	vec3 AC = C - A;
	vec3 ABcrossAC = cross(AB, AC);
	float D = dot(A, ABcrossAC);
	for (int mask = 0; mask < 7; mask++) {
		for (int j = 0; j < 3; j++) {
			if (mask & (1 << j)) {
				continue;
			}
			ray.origin = {
				(mask & 1) ? box.max.x : box.min.x,
				(mask & 2) ? box.max.y : box.min.y,
				(mask & 4) ? box.max.z : box.min.z
			};
			vec3 rayEnd = ray.origin;
			rayEnd[j] = box.max[j];
			if (signOf(dot(ray.origin, ABcrossAC) - D) != signOf(dot(rayEnd, ABcrossAC) - D)) {
				ray.dir = rayEnd - ray.origin;
				float gamma = 1.0000001;
				if (intersectTriangleFast(ray, A, B, C, gamma)) {
					return true;
				}
			}
		}
	}
	return false;
}

void TriangleMesh::Triangle::expandBox(BBox& box) {
	for (int c = 0; c < 3; c++) {
		box.add(owner->vertices[indices[c]]);
	}
}

vec3 TriangleMesh::Triangle::getCenter() {
	vec3 sum(0,0,0);
	for (int c = 0; c < 3; c++) {
		sum += owner->vertices[indices[c]];
	}
	return sum / 3.;
}

void TriangleMesh::onBeforeRender() {
	if (faces.size() < 50) {
		return;
	}

	if (!accelerator) {
		accelerator = makeDefaultAccelerator();
	}

	if (!accelerator->isBuilt()) {
		for (int c = 0; c < faces.size(); c++) {
			accelerator->addPrimitive(&faces[c]);
		}
		accelerator->build(IntersectionAccelerator::Purpose::Mesh);
	}
}

bool TriangleMesh::loadFromObj(const std::string& objPath) {
	tinyobj::attrib_t inattrib;
	std::vector<tinyobj::shape_t> inshapes;
	std::string error;
	std::vector<tinyobj::material_t> materials;
	const bool loadRes = tinyobj::LoadObj(&inattrib, &inshapes, &materials, &error, objPath.c_str(), nullptr);
	if (!loadRes) {
		printf("Error loading file \"%s\", \"%s\"", objPath.c_str(), error.c_str());
	}

	static_assert(sizeof(vec3) == sizeof(tinyobj::real_t) * 3, "next line avoids copy with type alias");
	vertices.swap(reinterpret_cast<std::vector<vec3>&>(inattrib.vertices));

	for (int c = 0; c < vertices.size(); c++) {
		box.add(vertices[c]);
	}

	for (int c = 0; c < inshapes.size(); c++) {
		int index = 0;
		bool skipMesh = false;
		const tinyobj::mesh_t &mesh = inshapes[c].mesh;

		const std::vector<unsigned char> &numFaceVertices = inshapes[c].mesh.num_face_vertices;
		for (int r = 0; r < numFaceVertices.size(); r++) {
			if (numFaceVertices[r] != 3) {
				skipMesh = true;
				break;
			}
		}
		if (skipMesh) {
			continue;
		}

		faces.reserve(faces.size() + numFaceVertices.size());
		for (int r = 0; r < numFaceVertices.size(); r++) {
			const Triangle face{
				mesh.indices[index++].vertex_index,
				mesh.indices[index++].vertex_index,
				mesh.indices[index++].vertex_index,
				this
			};
			faces.push_back(face);
		}
	}
	return true;
}


bool TriangleMesh::intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) {
	if (!box.testIntersect(ray)) {
		return false;
	}
	if (accelerator && accelerator->isBuilt()) {
		return accelerator->intersect(ray, tMin, tMax, intersection);
	}
	bool haveRes = false;
	for (int c = 0; c < faces.size(); c++) {
		haveRes = haveRes || faces[c].intersect(ray, tMin, tMax, intersection);
	}
	return haveRes;
}


