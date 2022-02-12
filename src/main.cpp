#define _CRT_SECURE_NO_WARNINGS

#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <atomic>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb_image_write.h"

#include "Threading.hpp"
#include "Material.h"
#include "Primitive.h"
#include "Image.hpp"
#include "Mesh.h"

/// Camera description, can be pointed at point, used to generate screen rays
struct Camera {
	const vec3 worldUp = {0, 1, 0};
	float aspect;
	vec3 origin;
	vec3 llc;
	vec3 left;
	vec3 up;

	void lookAt(float verticalFov, const vec3 &lookFrom, const vec3 &lookAt) {
		origin = lookFrom;
		const float theta = degToRad(verticalFov);
		float half_height = tan(theta / 2);
		const float half_width = aspect * half_height;

		const vec3 w = (origin - lookAt).normalized();
		const vec3 u = cross(worldUp, w).normalized();
		const vec3 v = cross(w, u);
		llc = origin - half_width * u - half_height * v - w;
		left = 2 * half_width * u;
		up = 2 * half_height * v;
	}

	Ray getRay(float u, float v) const {
		return Ray(origin, (llc + u * left + v * up - origin).normalized());
	}
};

vec3 raytrace(const Ray &r, Instancer &prims, int depth = 0) {
	Intersection data;
	if (prims.intersect(r, 0.001f, FLT_MAX, data)) {
		Ray scatter;
		Color attenuation;
		if (depth < MAX_RAY_DEPTH && data.material->shade(r, data, attenuation, scatter)) {
			const Color incoming = raytrace(scatter, prims, depth + 1);
			return attenuation * incoming;
		} else {
			return Color(0.f);
		}
	}
	const vec3 dir = r.dir;
	const float f = 0.5f * (dir.y + 1.f);
	return (1.f - f) * vec3(1.f) + f * vec3(0.5f, 0.7f, 1.f);
}

/// The whole scene description
struct Scene : Task {
	Scene() = default;
	Scene(const Scene &) = delete;
	Scene &operator=(const Scene &) = delete;

	int width = 640;
	int height = 480;
	int samplesPerPixel = 2;
	std::string name;
	std::atomic<int> renderedPixels;
	Instancer primitives;
	Camera camera;
	ImageData image;

	void onBeforeRender() {
		primitives.onBeforeRender();
	}

	void initImage(int w, int h, int spp) {
		image.init(w, h);
		width = w;
		height = h;
		samplesPerPixel = spp;
		camera.aspect = float(width) / height;
	}

	void addPrimitive(PrimPtr primitive) {
		primitives.addInstance(std::move(primitive));
	}

	void render(ThreadManager &tm) {
		runOn(tm);
	}

	void run(int threadIndex, int threadCount) override {
		const int total = width * height;
		const int incrementPrint = total / 100;
		for (int idx = threadIndex; idx < total; idx += threadCount) {
			const int r = idx / width;
			const int c = idx % width;

			Color avg(0);
			for (int s = 0; s < samplesPerPixel; s++) {
				const float u = float(c + randFloat()) / float(width);
				const float v = float(r + randFloat()) / float(height);
				const Ray &ray = camera.getRay(u, v);
				const vec3 sample = raytrace(ray, primitives);
				avg += sample;
			}

			avg /= samplesPerPixel;
			image(c, height - r - 1) = Color(sqrtf(avg.x), sqrtf(avg.y), sqrtf(avg.z));
			const int completed = renderedPixels.fetch_add(1, std::memory_order_relaxed);
			if (completed % incrementPrint == 0) {
				printf("\r%d%% ", int(float(completed) / float(total) * 100));
			}
		}
	}
};


void sceneExample(Scene &scene) {
	scene.name = "example";
	scene.initImage(800, 600, 4);
	scene.camera.lookAt(90.f, {-0.1f, 5, -0.1f}, {0, 0, 0});

	SharedPrimPtr mesh(new TriangleMesh(MESH_FOLDER "/cube.obj", MaterialPtr(new Lambert{Color(1, 0, 0)})));
	Instancer *instancer = new Instancer;
	instancer->addInstance(mesh, vec3(2, 0, 0));
	instancer->addInstance(mesh, vec3(0, 0, 2));
	instancer->addInstance(mesh, vec3(2, 0, 2));
	scene.addPrimitive(PrimPtr(instancer));

	const float r = 0.6f;
	scene.addPrimitive(PrimPtr(new SpherePrim{vec3(2, 0, 0), r, MaterialPtr(new Lambert{Color(0.8, 0.3, 0.3)})}));
	scene.addPrimitive(PrimPtr(new SpherePrim{vec3(0, 0, 2), r, MaterialPtr(new Lambert{Color(0.8, 0.3, 0.3)})}));
	scene.addPrimitive(PrimPtr(new SpherePrim{vec3(0, 0, 0), r, MaterialPtr(new Lambert{Color(0.8, 0.3, 0.3)})}));
}

void sceneManyHeavyMeshes(Scene &scene) {
	scene.name = "instanced-dragons";
	const int count = 50;

	scene.initImage(1280, 720, 10);
	scene.camera.lookAt(90.f, {0, 3, -count}, {0, 3, count});

	SharedMaterialPtr instanceMaterials[] = {
		SharedMaterialPtr(new Lambert{Color(0.2, 0.7, 0.1)}),
		SharedMaterialPtr(new Lambert{Color(0.7, 0.2, 0.1)}),
		SharedMaterialPtr(new Lambert{Color(0.1, 0.2, 0.7)}),
		SharedMaterialPtr(new Metal{Color(0.8, 0.1, 0.1), 0.3f}),
		SharedMaterialPtr(new Metal{Color(0.1, 0.7, 0.1), 0.6f}),
		SharedMaterialPtr(new Metal{Color(0.1, 0.1, 0.7), 0.9f}),
	};
	const int materialCount = std::size(instanceMaterials);

	auto getRandomMaterial = [instanceMaterials, materialCount]() -> SharedMaterialPtr {
		const int rng = int(randFloat() * materialCount);
		return instanceMaterials[rng];
	};

	SharedPrimPtr mesh(new TriangleMesh(MESH_FOLDER "/dragon.obj", MaterialPtr(new Lambert{Color(1, 0, 0)})));
	Instancer *instancer = new Instancer;

	instancer->addInstance(mesh, vec3(0, 2.5, -count + 1), 0.08f, getRandomMaterial());

	for (int c = -count; c <= count; c++) {
		for (int r = -count; r <= count; r++) {
			instancer->addInstance(mesh, vec3(c, 0, r), 0.05f, getRandomMaterial());
			instancer->addInstance(mesh, vec3(c, 6, r), 0.05f, getRandomMaterial());
		}
	}

	scene.addPrimitive(PrimPtr(instancer));
}

void sceneManySimpleMeshes(Scene &scene) {
	scene.name = "instanced-cubes";
	const int count = 20;

	scene.initImage(800, 600, 2);
	scene.camera.lookAt(90.f, {0, 2, count}, {0, 0, 0});

	SharedPrimPtr mesh(new TriangleMesh(MESH_FOLDER "/cube.obj", MaterialPtr(new Lambert{Color(1, 0, 0)})));
	Instancer *instancer = new Instancer;

	for (int c = -count; c <= count; c++) {
		for (int r = -count; r <= count; r++) {
			instancer->addInstance(mesh, vec3(c, 0, r), 0.5f);
		}
	}

	scene.addPrimitive(PrimPtr(instancer));
}

void sceneHeavyMesh(Scene &scene) {
	scene.name = "dragon";
	scene.initImage(800, 600, 4);
	scene.camera.lookAt(90.f, {8, 10, 7}, {0, 0, 0});
	scene.addPrimitive(PrimPtr(new TriangleMesh(MESH_FOLDER "/dragon.obj", MaterialPtr(new Lambert{Color(0.2, 0.7, 0.1)}))));
}

int main(int argc, char *argv[]) {

	void (*sceneCreators[])(Scene &) = {
		sceneExample,
		sceneHeavyMesh,
		sceneManySimpleMeshes,
		sceneManyHeavyMeshes
	};

	puts("> There are 4 scenes (0,1,2,3) to render");
	puts("> Pass no arguments to render the example scene (index 0)");
	puts("> Pass one argument, index of the scene to render or -1 to render all");
	puts("");

	const int sceneCount = std::size(sceneCreators);
	int renderCount = sceneCount;
	int firstScene = 0;
	if (argc == 1) {
		renderCount = 1;
		puts("No arguments, will render only example scene");
	} else {
		const int arg = atoi(argv[1]);
		if (arg == -1) {
			renderCount = sceneCount;
			firstScene = 0;
		} else if (arg >= 1 && arg < sceneCount) {
			firstScene = arg;
			renderCount = sceneCount;
		}
	}

	const int threadCount = std::max<unsigned>(std::thread::hardware_concurrency() - 1, 1);
	ThreadManager tm(threadCount);
	tm.start();

	for (int c = 0; c < renderCount; c++) {
		const int sceneIndex = c + firstScene;
		Scene scene;
		printf("Loading scene...\n");
		sceneCreators[sceneIndex](scene);
		printf("Preparing \"%s\" scene...\n", scene.name.c_str());
		scene.onBeforeRender();
		printf("Starting rendering\n");
		{
			Timer timer;
			scene.render(tm);
			printf("Render time: %gms\n", Timer::toMs<float>(timer.elapsedNs()));
		}
		const std::string resultImage = scene.name + ".png";
		printf("Saving image to \"%s\"...\n", resultImage.c_str());
		const PNGImage &png = scene.image.createPNGData();
		const int success = stbi_write_png(resultImage.c_str(), scene.width, scene.height, PNGImage::componentCount(), png.data.data(), sizeof(PNGImage::Pixel) * scene.width);
		if (success == 0) {
			printf("Failed to write image \"%s\"\n", resultImage.c_str());
		}
		puts("");
	}

	printf("Done.");
	tm.stop();

	return 0;
}
