#include "Primitive.h"
#include "Threading.hpp"
#include <cstring>
#include <algorithm>

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
		if (currentDepth >= MAX_DEPTH || n->primitives.size() <= (unsigned long int) MIN_PRIMITIVES) {
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
			for (unsigned long int r = 0; r < n->primitives.size(); r++) {
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
		for (unsigned long int c = 0; c < root->primitives.size(); c++) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root);
		printf(" done in %ldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);
	}

	bool intersect(Node *n, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			for (unsigned long int c = 0; c < n->primitives.size(); c++) {
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
		Node() : left(nullptr), right(nullptr), splitAxis(-1) {}
		bool isLeaf() {
			return children[0] == nullptr;
		}
	};

	// faster intersection tree node;
	// left child will always be next in the array, right child is a index in the nodes array.
	struct FastNode {
		BBox box;
		long unsigned int right; // 4e9 is a puny number of primitives => long, not int
		Intersectable* *primitives;
		char splitAxis;
		bool isLeaf() {
			// 0th node will always be the root so no node points to it as its right child
			return right == 0;
		}
	};

	// the simplest possible allocator that could be
	template <class T>
		class StackAllocator {
			T *buff = nullptr;
			long unsigned int maxSize = 0;
			long unsigned size = 0;

			public:
			StackAllocator() {};

			void init(size_t maxSize) {
				if(buff != nullptr) delete buff;

				this->maxSize = maxSize;
				size = 0;
				buff = new T[maxSize];
				memset(buff, 0, maxSize * sizeof(T));
			}

			T *alloc(size_t size) {
				assert(size + this->size <= maxSize);
				T *res = &(buff[this->size]);
				this->size += size;
				return res;
			}

			~StackAllocator() {
				if(buff != nullptr) {
					delete [] buff;
					buff = nullptr;
				}
			}

			long unsigned int getSize() {return size;}
		};
	
	// all primitives added
	std::vector<Intersectable*> allPrimitives;
	// root of the construction tree
	Node *root = nullptr;
	// nodes of the fast traversal tree
	std::vector<FastNode> fastNodes;
	// the primitives sorted for fast traversal
	StackAllocator<Intersectable*> fastTreePrimitives;
	bool built = false;
	// cost for traversing a parent node. It is assumed that the intersection cost with a primitive is 1.0
	static constexpr float SAH_TRAVERSAL_COST = 0.125;
	// the number of splits SAH will try.
	static constexpr int SAH_TRY_COUNT = 5;
	static constexpr int MAX_DEPTH = 50;
	static constexpr int MIN_PRIMITIVES_COUNT = 4;
	// when a node has less than that number of primitives it will sort them and always split in the middle
	// 
	// theoretically: 
	// the higher this is, the better the tree and the slower its construction
	// 
	// practically: 
	// when setting this to something higher, the tree construction time changes by verry little
	// and the render time goes up. When this is set to 1e6 (basically always sort), makes 
	// rendering about 30% slower than when it is 0 (never split perfectly). (on the instanced dragons scene) 
	// Perfect splits clearly produce shallower trees, which should make rendering faster ... but it doesn't.
	//
	// I guess SAH is too good
	static constexpr int PERFECT_SPLIT_THRESHOLD = 0;

	int depth = 0;
	int leafSize = 0;
	long int leavesCount = 0;
	long int nodeCount = 0;
	long int primitivesCount = 0;

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
		clearConstructionTree();
		root = 0;
		depth = 0;
		nodeCount = 0;
		leafSize = 0;
	}

	// clears the construction tree.
	void clearConstructionTree() {
		clear(root);
		delete(root);
	}

	void build(Node *node, int depth) {
		if(depth > MAX_DEPTH || node->primitives.size() <= MIN_PRIMITIVES_COUNT) {
			leafSize = std::max((int)(node->primitives.size()), leafSize);
			++ leavesCount;
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
		node->splitAxis = maxAxis;
		
		// choose splitting algorithm
		if(node->primitives.size() < PERFECT_SPLIT_THRESHOLD) {
			unsigned long int size = node->primitives.size();
			// sorts so that the middle element is in its place, all others are in sorted order relative to it
			std::nth_element(node->primitives.begin(), node->primitives.begin() + size * 0.5, node->primitives.end(),
							 [&](Intersectable* &a, Intersectable* &b) {
								 return a->getCenter()[maxAxis] < b->getCenter()[maxAxis];
							 });
			node->left = new Node();
			node->right = new Node();
			nodeCount +=2;
			
			// split in half
			for(unsigned long int i = 0; i < size; ++ i) {
				bool ind = i > (size / 2 - 1);
				node->primitives[i]->expandBox(node->children[ind]->box);
				node->children[ind]->primitives.push_back(node->primitives[i]);
			}
		} else {
			long int noSplitSAH = node->primitives.size();

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

			// create a leaf when the node can't be split effectively
			if(bestSAH > noSplitSAH) {
				leafSize = std::max((int)(node->primitives.size()), leafSize);
				++ leavesCount;
				return;
			}
			
			// position of the split plane. lerp between min and max
			const float split = node->box.min[maxAxis] * bestRatio + node->box.max[maxAxis] * (1 - bestRatio);
			// distibute the primitives to the child nodes
			node->left = new Node();
			node->right = new Node();
			nodeCount +=2;
			for(Intersectable *obj : node->primitives) {
				int ind = obj->getCenter()[maxAxis] > split;
				obj->expandBox(node->children[ind]->box);
				node->children[ind]->primitives.push_back(obj);
			}
		}

		build(node->left, depth + 1);
		build(node->right, depth + 1);
		node->primitives.clear();
	}

	void build(Purpose purpose) override {
		// purpose is ignored. what works best for triangles seems to also work best for objects
		printf("Building BVH tree with %d primitives... ", int(allPrimitives.size()));
		Timer timer;

		primitivesCount = allPrimitives.size();

		root = new Node();
		root->primitives.swap(allPrimitives);
		for(unsigned long int c = 0; c < root->primitives.size(); ++c) {
			root->primitives[c]->expandBox(root->box);
		}
	
		// build both trees
		build(root, 0);
		buildFastTree();

		// construction tree is no longer needed
		clearConstructionTree();

		built = true;
		printf(" done in %ldms, nodes: %ld, leaves: %ld, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodeCount, leavesCount, depth, leafSize);
	}

	bool isBuilt() const override { return built; }

	bool intersect(long unsigned int nodeIndex, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
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

			// I know that the next section is almost unreadable, 
			// won't be much better if I replace it with a lot of copy-pasted if-statements

			// the distances to both children boxes
			float dist[2];
			long unsigned int childIndices[2] = {nodeIndex + 1, node->right};
			bool testIntersect[2] = {
				fastNodes[childIndices[0]].box.testIntersect(ray, dist[0]),
				fastNodes[childIndices[1]].box.testIntersect(ray, dist[1])};

			int direction = ray.dir[node->splitAxis] > 0.f;

			if(testIntersect[!direction]) {
				if(intersect(childIndices[!direction], ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
			// if the closest found intersection is farther than the intersection
			// with the second box, traverse it, otherwise skip it
			if(tMax > dist[direction] && testIntersect[direction]) {
				if(intersect(childIndices[direction], ray, tMin, tMax, intersection)) {
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

	// computes the SAH cost for a split on a given axis.
	// ratio equals the size of the left child on the chosen axis over
	// the size of the parent node size on that axis.
	float costSAH(const Node *node, int axis, float ratio) {
		// effectively a lerp between the min and max of the box
		const float split = node->box.min[axis] * ratio + node->box.max[axis] * (1 - ratio);
		long int count[2] = {0,0};
		BBox boxes[2];
		// count indices and merge bounding boxes in both children
		for(Intersectable *obj : node->primitives) {
			int ind = (obj->getCenter()[axis] > split);
			++count[ind];
			obj->expandBox(boxes[ind]);
		}
		// just the formula
		float s0 = node->box.surfaceArea();
		float s[2] = {
			count[0] ? boxes[0].surfaceArea() : 0,
			count[1] ? boxes[1].surfaceArea() : 0};
		return SAH_TRAVERSAL_COST + (s[0] * count[0] + s[1] * count[1]) / s0;
	}

	// builds a tree for fast traversal
	void buildFastTree() {
		// no reallocations whould happen
		fastNodes.reserve(nodeCount);

		// every leaf will have a list of primitives that ends with a null pointer.
		fastTreePrimitives.init(primitivesCount + leavesCount);

		// the other function expects the parent node to already have been pushed to the vector
		fastNodes.push_back(makeFastLeaf(root));
		// leaves must not be traced down
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

	// copies the data from a Node to a FastNode when constructing the traversal tree
	FastNode makeFastLeaf(Node *node) {
		if(node->isLeaf()) {
			// This should have sped up things because continuous allocation should be better for cache.
			// But it has no effect. It looks like the access of primitives is random enough that
			// it always generates a cache miss.
			Intersectable** primitives = fastTreePrimitives.alloc(node->primitives.size() + 1);
			//Intersectable** primitives = new Intersectable*[node->primitives.size() + 1];
			memcpy(primitives, &(node->primitives[0]), node->primitives.size() * sizeof(Intersectable*));
			return FastNode{node->box, 0, primitives, node->splitAxis};
		} else {
			return FastNode{node->box, 0, nullptr, node->splitAxis};
		}
	}

};

// BENCHMARKING:
// tree type		scene				tree construction time (ms)			render time (ms)
// oct				dragon				1576								1171
// oct				instancedCubes		8									2313
// oct				instancedDragons	1572 + 9							109969
// bvh				dragon				209									266
// bvh				instancedCubes		1									170
// bvh				instancedDragons	210 + 21							8726
//
//
// NOTE:
// Домашно от Борис Василев
// Направих промени и в другите файлове в проекта. Форкнал съм проекта в Гитхъб, така че
// там могат лесно да се видят промените. Знам, че SAH изчисленията могат да бъдат
// оптимизирани още малко (центъра на всяка кутия се изчислява при всяко сравнение и подобни).
// Но дори и да се направи максимално ефективно, едва ли ще свали повече от 20% от времето 
// за строеж на дървото, което на фона на 200ms няма да е голяма работа, осбено имайки пред вид, 
// че в повечето случаи ще трябва да се рендърват повече от 1 кадър със повече от 2 лъча на пиксел,
// а виждаме, че по-сложната сцена вече отнема >40 пъти повече време.
//
// общ резултат: 
// 5-7x по-добро време за конструкция на дървото
// 5-10x по-добро време за трасиране на лъчите
//
// С нетърпение очаквам коментара Ви. Надявам се скоро да имаме възможността да работим заедно отново.

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//return AcceleratorPtr(new KDTree());
	return AcceleratorPtr(new BVHTree());
	//return AcceleratorPtr(new OctTree());
}

