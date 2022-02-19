#include "Primitive.h"
#include "Threading.hpp"
#include <cstring>

struct OctTree : IntersectionAccelerator {
	struct Node {
		BBox box;
		Node *children[8] = {nullptr, };
		std::vector<Intersectable*> primitives;
		bool isLeaf() const {
			return children[0] == nullptr;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	int depth = 0;
	int leafSize = 0;
	int nodes = 0;
	int MAX_DEPTH = 35;
	int MIN_PRIMITIVES = 10;

	void clear(Node *n) {
		if (!n) {
			return;
		}

		for (int c = 0; c < 8; c++) {
			clear(n->children[c]);
			delete n->children[c];
		}
	}

	void clear() {
		clear(root);
		allPrimitives.clear();
	}

	void addPrimitive(Intersectable* prim) override {
		allPrimitives.push_back(prim);
	}

	void build(Node *n, int currentDepth = 0) {
		if (currentDepth >= MAX_DEPTH || n->primitives.size() <= MIN_PRIMITIVES) {
			leafSize = std::max(leafSize, int(n->primitives.size()));
			return;
		}

		depth = std::max(depth, currentDepth);
		BBox childBoxes[8];
		n->box.octSplit(childBoxes);

		for (int c = 0; c < 8; c++) {
			Node *& child = n->children[c];
			child = new Node;
			nodes++;
			memset(child->children, 0, sizeof(child->children));
			child->box = childBoxes[c];
			for (int r = 0; r < n->primitives.size(); r++) {
				if (n->primitives[r]->boxIntersect(child->box)) {
					child->primitives.push_back(n->primitives[r]);
				}
			}
			if (child->primitives.size() == n->primitives.size()) {  
				build(child, MAX_DEPTH + 1);
			} else {
				build(child, currentDepth + 1);
			}
		}
		n->primitives.clear();
	}

	void build(Purpose purpose) override {
		const char *treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			MIN_PRIMITIVES = 4;
			treePurpose = " instances";
		} else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			MIN_PRIMITIVES = 20;
			treePurpose = " mesh";
		}

		if (root) {
			clear(root);
			delete root;
		}

		printf("Building %s oct tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		nodes = leafSize = depth = 0;
		root = new Node();
		root->primitives.swap(allPrimitives);
		for (int c = 0; c < root->primitives.size(); c++) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root);
		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);
	}

	bool intersect(Node *n, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			for (int c = 0; c < n->primitives.size(); c++) {
				if (n->primitives[c]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		} else {
			for (int c = 0; c < 8; c++) {
				if (n->children[c]->box.testIntersect(ray)) {
					if (intersect(n->children[c], ray, tMin, tMax, intersection)) {
						tMax = intersection.t;
						hasHit = true;
					}
				}
			}
		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		return intersect(root, ray, tMin, tMax, intersection);
	}

	bool isBuilt() const override {
		return root != nullptr;
	}

	~OctTree() override {
		clear();
	}
};

/// TODO: Implement one/both or any other acceleration structure and change makeDefaultAccelerator to create it
struct KDTree : IntersectionAccelerator {
	void addPrimitive(Intersectable *prim) override {}
	void clear() override {}
	void build(Purpose purpose) override {}
	bool isBuilt() const override { return false; }
	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override { return false; }
};


struct BVHTree : IntersectionAccelerator {
	// Node structure
	// does not need a parent pointer since the intersection implementation will be recursive
	// I can afford that since the code will run on CPU which will not suffer from it.
	// This will simplify the implementation a lot.
	struct Node {
		BBox box;
		Node *children[2] = {nullptr, nullptr};
		std::vector<Intersectable*> primitives;
		bool isLeaf() {
			return children[0] == nullptr;
		}
	};

	// faster intersection tree node;
	// left child will always be next in the array, right child is a pointer.
	// Vectors allocate memory on the heap and so are slow to access
	// so will use a static array with the maximum leaf size.
	// This is not 100% efficient because not all leaves will have the maximum number of primitives
	// also, a vector is 24 bytes long, while an array with 4 or more pointers will be at least
	// 32 bytes. 
	// So a vector will result in less cache misses when traversing non-leaf nodes due to smaller 
	// size of the struct, but accessing primitives in leaves will jump to a pointer
	// so it will be a tradeoff, TODO: benchmark this
	template<int leafSize>
	struct FastNode {
		BBox box;
		Node* right;
		Intersectable* primitives[leafSize];
		bool isLeaf() {
			return right == nullptr;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	bool built = false;
	static constexpr float SAH_TRAVERSAL_COST = 0.125;
	static constexpr float SAH_INTERSECTION_COST = 1;
	static constexpr int SAH_TRY_COUNT = 5;
	static constexpr int MAX_DEPTH = 60;
	static constexpr int MIN_PRIMITIVES_COUNT = 4;
	
	int depth = 0;
	int leaf_primitives = 0;
	int nodes = 0;

	void addPrimitive(Intersectable *prim) override {
		allPrimitives.push_back(prim);
	}

	void clear(Node *node) {
		if(node == nullptr) return;
		for(int i = 0; i < 2; i ++) {
			clear(node->children[i]);
			delete node->children[i];
		}
	}

	void clear() override {
		allPrimitives.clear();
		built = false;
		clear(root);
		delete root;
		depth = 0;
		nodes = 0;
		leaf_primitives = 0;
	}
	
	void build(Node *node, int depth) {
		if(depth > MAX_DEPTH || node->primitives.size() <= MIN_PRIMITIVES_COUNT) {
			leaf_primitives = std::max((int)(node->primitives.size()), leaf_primitives);
			return;
		}
		this->depth = std::max(depth, this->depth);

		//vec3 size = node->box.max - node->box.min;
		
		// get a bounding box of all centroids
		BBox centerBox;
		for(Intersectable *obj : node->primitives) {
			centerBox.add(obj->getCenter());
		}
		vec3 size = centerBox.max - centerBox.min;

		// find the axis on which the box is largest
		int maxAxis = -1;
		float maxAxisValue = -FLT_MAX;
		for(int i = 0; i < 3; ++i) {
			if(maxAxisValue < size[i]) {
				maxAxis = i;
				maxAxisValue = size[i];
			}
		}
		
		// TODO: this may not be needed when a minimum primitives count is specified
		float noSplitSAH = node->primitives.size();

		// try evenly distributed splits with SAH
		float bestSAH = FLT_MAX, bestRatio = -1;
		for(int i = 0; i < SAH_TRY_COUNT; ++ i) {
			float ratio = float(i + 1.) / float(SAH_TRY_COUNT + 1);	
			float sah = costSAH(node, maxAxis, ratio);
			if(bestSAH > sah) {
				bestSAH = sah;
				bestRatio = ratio;
			}
		}
	
		if(bestSAH > noSplitSAH) {
			leaf_primitives = std::max((int)(node->primitives.size()), leaf_primitives);
			return;
		}
		//bestRatio = 0.5;

		const float split = node->box.min[maxAxis] * bestRatio + node->box.max[maxAxis] * (1 - bestRatio);
		// distibute the primitives to the child nodes
		node->children[0] = new Node();
		node->children[1] = new Node();
		nodes +=2;
		for(Intersectable *obj : node->primitives) {
			int ind = obj->getCenter()[maxAxis] > split; // index is 0 or 1 depending on that boolean value
			obj->expandBox(node->children[ind]->box);
			node->children[ind]->primitives.push_back(obj);
		}

		for(int i = 0; i < 2; i ++) { 
			build(node->children[i], depth + 1);
		}
		node->primitives.clear();
	}

	void build(Purpose purpose) override {
		printf("Building oct tree with %d primitives... ", int(allPrimitives.size()));
		Timer timer;

		root = new Node();
		root->primitives.swap(allPrimitives);
		for (int c = 0; c < root->primitives.size(); ++c) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root, 0);
		built = true;

		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leaf_primitives);
		
		printf("leaf size 0: %d\n", sizeof(FastNode<0>));
		printf("leaf size 1: %d\n", sizeof(FastNode<1>));
		printf("leaf size 2: %d\n", sizeof(FastNode<2>));
		printf("leaf size 3: %d\n", sizeof(FastNode<3>));
		printf("vector: %d\n", sizeof(std::vector<Intersectable*>));
	}

	bool isBuilt() const override { return built; }
	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override { return false; }

	float costSAH(const Node *node, int axis, float ratio) {
		const float split = node->box.min[axis] * ratio + node->box.max[axis] * (1 - ratio);
		int count[2] = {0,0};
		BBox boxes[2];
		boxes[0] = BBox();
		boxes[1] = BBox();
		for(Intersectable *obj : node->primitives) {
			int ind = (obj->getCenter()[axis] > split);
			++count[ind];
			obj->expandBox(boxes[ind]);
		}
		float s0 = node->box.surfaceArea();
		float s[2] = {count[0] ? boxes[0].surfaceArea() : 0, count[1] ? boxes[1].surfaceArea() : 0};
		return SAH_TRAVERSAL_COST + (s[0] * count[0] + s[1] * count[1]) / s0;
	}

};

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//return AcceleratorPtr(new KDTree());
	return AcceleratorPtr(new BVHTree());
	//return AcceleratorPtr(new OctTree());
}

