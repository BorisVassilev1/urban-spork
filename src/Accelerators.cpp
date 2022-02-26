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
		union {
			Node *children[2];
			struct {
				Node *left = nullptr, *right = nullptr;
			};
		};
		std::vector<Intersectable*> primitives;
		char splitAxis;
		bool isLeaf() {
			return children[0] == nullptr;
		}
	};

	// faster intersection tree node;
	// left child will always be next in the array, right child is a index in the nodes array.
	struct FastNode {
		BBox box;
		unsigned int right;
		Intersectable* *primitives;
		char splitAxis;
		bool isLeaf() {
			// 0th node will always be the root so no node points to it as its right child
			return right == 0;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	std::vector<FastNode> fastNodes;
	bool built = false;
	static constexpr float SAH_TRAVERSAL_COST = 0.125;
	static constexpr float SAH_INTERSECTION_COST = 1;
	static constexpr int SAH_TRY_COUNT = 5;
	static constexpr int MAX_DEPTH = 60;
	static constexpr int MIN_PRIMITIVES_COUNT = 4;

	int depth = 0;
	int leafSize = 0;
	int nodeCount = 0;

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
		root = 0;
		depth = 0;
		nodeCount = 0;
		leafSize = 0;
		
		for(FastNode &node : fastNodes) {
			delete node.primitives;
		}
	}

	void build(Node *node, int depth) {
		if(depth > MAX_DEPTH || node->primitives.size() <= MIN_PRIMITIVES_COUNT) {
			leafSize = std::max((int)(node->primitives.size()), leafSize);
			return;
		}
		this->depth = std::max(depth, this->depth);

		// get a bounding box of all centroids
		BBox centerBox;
		for(Intersectable *obj : node->primitives) {
			centerBox.add(obj->getCenter());
		}
		vec3 size = centerBox.max - centerBox.min;

		// find the axis on which the box is largest
		char maxAxis = -1;
		float maxAxisValue = -FLT_MAX;
		for(int i = 0; i < 3; ++i) {
			if(maxAxisValue < size[i]) {
				maxAxis = i;
				maxAxisValue = size[i];
			}
		}

		// TODO: when primitive count is small, just sort and split in the middle
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
			leafSize = std::max((int)(node->primitives.size()), leafSize);
			return;
		}

		node->splitAxis = maxAxis;
		const float split = node->box.min[maxAxis] * bestRatio + node->box.max[maxAxis] * (1 - bestRatio);
		// distibute the primitives to the child nodes
		node->left = new Node();
		node->right = new Node();
		nodeCount +=2;
		for(Intersectable *obj : node->primitives) {
			int ind = obj->getCenter()[maxAxis] > split; // index is 0 or 1 depending on that boolean value
			obj->expandBox(node->children[ind]->box);
			node->children[ind]->primitives.push_back(obj);
		}

		build(node->left, depth + 1);
		build(node->right, depth + 1);
		node->primitives.clear();
	}

	void build(Purpose purpose) override {
		printf("Building BVH tree with %d primitives... ", int(allPrimitives.size()));
		Timer timer;

		root = new Node();
		root->primitives.swap(allPrimitives);
		for(int c = 0; c < root->primitives.size(); ++c) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root, 0);
		built = true;
		
		buildFastTree();

		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodeCount, depth, leafSize);
	}

	bool isBuilt() const override { return built; }

	bool intersect(int nodeIndex, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;
		FastNode *node = &(fastNodes[nodeIndex]);
		
		if(node->isLeaf()) {
			//printf("intersecting leaf: %d, %d\n", nodeIndex, node->primitives[0] == nullptr);
			// iterate either to a invalid pointer or to the max leaf size
			for(int i = 0; i < leafSize && node->primitives[i] != nullptr; i++) {
				// copypasta from the octree
				if(node->primitives[i]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		} else {
			// If the ray moves towards the positive direction, then will traverse in order left->right
			// and right->left if else.
			// First intersect with both child bounding boxes. Intersection with things inside the second box can 
			// be skipped if distance to intersection found in the first one is closer than that with the second box
			//				 left 		 right 
			//             +---------+----+--------+
			//       ray   |         |    |        |
			//     --------+-->/\    |    |   /\   |
			//             |  /--\   |    |  /--\  |
			//             |         |    |        |
			//             +---------+----+        |
			//                       |             |
			//                       +-------------+
			
			if(fastNodes[nodeIndex+1].box.testIntersect(ray)) {
				if(intersect(nodeIndex+1, ray, tMin, tMax, intersection)) {
					//printf("%d hit %d\n", nodeIndex, nodeIndex + 1);
					tMax = intersection.t;
					hasHit = true;
				}
			}
			if(fastNodes[node->right].box.testIntersect(ray)) {
				if(intersect(node->right, ray, tMin, tMax, intersection)) {
					//printf("%d hit %d\n", nodeIndex, node->right);
					tMax = intersection.t;
					hasHit = true;
				}
			}

		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		if(fastNodes[0].box.testIntersect(ray)) {
			return intersect(0, ray, tMin, tMax, intersection);
		} else return false;
	}


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


	void buildFastTree() {
		fastNodes.reserve(nodeCount);

		fastNodes.push_back(makeFastLeaf(root));
		if(!(root->isLeaf())) {
			buildFastTree(root, fastNodes);
		}
	}

	void buildFastTree(Node *node, std::vector<FastNode> &allNodes) {
		int parentIndex = allNodes.size() - 1;

		// insert the left child
		allNodes.push_back(makeFastLeaf(node->left));

		// trace down left
		if(!(node->left->isLeaf())) {
			buildFastTree(node->left, allNodes);
		}

		// add right child
		allNodes.push_back(makeFastLeaf(node->right));

		// write to the parent right pointer
		allNodes[parentIndex].right = allNodes.size() - 1;

		// trace down right
		if(!(node->right->isLeaf())) {
			buildFastTree(node->right, allNodes);
		}
	}

	FastNode makeFastLeaf(Node *node) {
		if(node->isLeaf()) {
			// TODO: this is slow because of heap allocation
			// this should point to an array somewhere
			Intersectable** primitives = new Intersectable*[node->primitives.size() + 1];
			memset(primitives, (long int)nullptr, (node->primitives.size() + 1) * sizeof(Intersectable*));
			memcpy(primitives, &(node->primitives[0]), node->primitives.size() * sizeof(Intersectable*));
			return FastNode{node->box, 0, primitives, node->splitAxis};
		} else {
			return FastNode{node->box, 0, nullptr, node->splitAxis};
		}
	}

};

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//return AcceleratorPtr(new KDTree());
	//return AcceleratorPtr(new BVHTree());
	return AcceleratorPtr(new OctTree());
}

