/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  ydlidar_math_kmeans_H
#define  ydlidar_math_kmeans_H
// Includes
#include <iostream>
#include <sstream>
#include <time.h>
#include <assert.h>
#include <stddef.h>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <stdint.h>
#include <string.h>

#ifdef HAVE_MALLOC_H
# include <malloc.h>
#elif defined(HAVE_MALLOC_MALLOC_H)
# include <malloc/malloc.h>
#endif

#include <memory.h>
#include <cstdlib>

using namespace std;

// The data-type used for a single coordinate for points
typedef double Scalar;


namespace ydlidar {
namespace math {

/** CArrayNumeric is an array for numeric types supporting several mathematical operations (actually, just a wrapper on std::array<T,N>)
  * \sa CArrayFloat, CArrayDouble, CArray
  */
template <typename T, std::size_t N>
class CArrayNumeric : public std::array<T, N> {
 public:
  typedef T                    value_type;
  typedef std::array<T, N> Base;


  CArrayNumeric() {}  //!< Default constructor
  /** Constructor from initial values ptr[0]-ptr[N-1] */
  CArrayNumeric(const T *ptr) : std::array<T, N>(ptr) {}

  /** Initialization from a vector-like source, that is, anything implementing operator[]. */
  template <class ARRAYLIKE>
  explicit CArrayNumeric(const ARRAYLIKE &obj) : std::array<T, N>(obj) {}

};

// --------------  Partial specializations of CArrayNumeric -----------

/** A partial specialization of CArrayNumeric for float numbers.
  * \sa CArrayNumeric, CArray */
template <std::size_t N>
class CArrayFloat : public CArrayNumeric<float, N> {
 public:
  typedef CArrayNumeric<float, N> Base;
  typedef CArrayFloat<N> mrpt_autotype;

  CArrayFloat() {}  //!< Default constructor
  CArrayFloat(const float *ptr) : CArrayNumeric<float, N>
    (ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]


  /** Initialization from a vector-like source, that is, anything implementing operator[]. */
  template <class ARRAYLIKE>
  explicit CArrayFloat(const ARRAYLIKE &obj) : CArrayNumeric<float, N>(obj) {}
};

/** A partial specialization of CArrayNumeric for double numbers.
  * \sa CArrayNumeric, CArray */
template <std::size_t N>
class CArrayDouble : public CArrayNumeric<double, N> {
 public:
  typedef CArrayNumeric<double, N> Base;
  typedef CArrayDouble<N> mrpt_autotype;

  CArrayDouble() {}  //!< Default constructor
  CArrayDouble(const double *ptr) : CArrayNumeric<double, N>
    (ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]


  /** Initialization from a vector-like source, that is, anything implementing operator[]. */
  template <class ARRAYLIKE>
  explicit CArrayDouble(const ARRAYLIKE &obj) : CArrayNumeric<double, N>(obj) {}
};

/** A partial specialization of CArrayNumeric for int numbers.
  * \sa CArrayNumeric, CArray */
template <std::size_t N>
class CArrayInt : public CArrayNumeric<int, N> {
 public:
  typedef CArrayNumeric<int, N> Base;
  typedef CArrayInt<N> mrpt_autotype;

  CArrayInt() {}  //!< Default constructor
  CArrayInt(const int *ptr) : CArrayNumeric<int, N>
    (ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
};

/** A partial specialization of CArrayNumeric for unsigned int numbers.
  * \sa CArrayNumeric, CArray */
template <std::size_t N>
class CArrayUInt : public CArrayNumeric<unsigned int, N> {
 public:
  typedef CArrayNumeric<unsigned int, N> Base;
  typedef CArrayUInt<N> mrpt_autotype;

  CArrayUInt() {}  //!< Default constructor
  CArrayUInt(const unsigned int *ptr) : CArrayNumeric<unsigned int, N>
    (ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
};

// Fwrd. decl:
template<class T> class aligned_allocator;


/** Helper types for STL containers with std memory allocators.  */
template <class TYPE1, class TYPE2 = TYPE1>
struct aligned_containers {
  typedef std::pair<TYPE1, TYPE2> pair_t;
  typedef std::vector<TYPE1, std::allocator<TYPE1> > vector_t;
  typedef std::deque<TYPE1,  std::allocator<TYPE1> > deque_t;
  typedef std::list<TYPE1, std::allocator<TYPE1> > list_t;
  typedef std::map<TYPE1, TYPE2, std::less<TYPE1>, std::allocator<std::pair<const TYPE1, TYPE2> > >
  map_t;
  typedef std::multimap<TYPE1, TYPE2, std::less<TYPE1>, std::allocator<std::pair<const TYPE1, TYPE2> > >
  multimap_t;
};



// Point creation and deletion
inline Scalar *PointAllocate(int d) {
  return (Scalar *)malloc(d * sizeof(Scalar));
}

inline void PointFree(Scalar *p) {
  free(p);
}

inline void PointCopy(Scalar *p1, const Scalar *p2, int d) {
  memcpy(p1, p2, d * sizeof(Scalar));
}

// Point vector tools
inline void PointAdd(Scalar *p1, const Scalar *p2, int d) {
  for (int i = 0; i < d; i++) {
    p1[i] += p2[i];
  }
}

inline void PointScale(Scalar *p, Scalar scale, int d) {
  for (int i = 0; i < d; i++) {
    p[i] *= scale;
  }
}

inline Scalar PointDistSq(const Scalar *p1, const Scalar *p2, int d) {
  Scalar result = 0;

  for (int i = 0; i < d; i++) {
    result += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  }

  return result;
}

// Assertions
// ==========

int __KMeansAssertionFailure(const char *file, int line, const char *expression) {
  cout << "ASSERTION FAILURE, " << file << " line " << line << ":" << endl;
  cout << "  " << expression << endl;
  exit(-1);
}

// Comment out ENABLE_KMEANS_ASSERTS to turn off ASSERTS for added speed.
#define ENABLE_KMEANS_ASSERTS
#ifdef ENABLE_KMEANS_ASSERTS
int __KMeansAssertionFailure(const char *file, int line, const char *expression);
#define KM_ASSERT(expression) \
      (void)((expression) != 0? 0 : __KMeansAssertionFailure(__FILE__, __LINE__, #expression))
#else
#define KM_ASSERT(expression)
#endif



// Miscellaneous utilities
// =======================

// Returns a random integer chosen uniformly from the range [0, n-1]. Note that RAND_MAX could be
// less than n. On Visual Studio, it is only 32767. For larger values of RAND_MAX, we need to be
// careful of overflow.
inline int GetRandom(int n) {
  int u = rand() * RAND_MAX + rand();
  return ((u % n) + n) % n;
}


class KmTree {
 public:
  // Constructs a tree out of the given n data points living in R^d.
  explicit KmTree(int n, int d, Scalar *points): n_(n), d_(d), points_(points) {
    int node_size = sizeof(Node) + d_ * 3 * sizeof(Scalar);
    node_data_ = (char *)malloc((2 * n - 1) * node_size);
    point_indices_ = (int *)malloc(n * sizeof(int));

    for (int i = 0; i < n; i++) {
      point_indices_[i] = i;
    }

    KM_ASSERT(node_data_ != 0 && point_indices_ != 0);

    // Calculate the bounding box for the points
    Scalar *bound_v1 = PointAllocate(d_);
    Scalar *bound_v2 = PointAllocate(d_);
    KM_ASSERT(bound_v1 != 0 && bound_v2 != 0);
    PointCopy(bound_v1, points, d_);
    PointCopy(bound_v2, points, d_);

    for (int i = 1; i < n; i++)
      for (int j = 0; j < d; j++) {
        if (bound_v1[j] > points[i * d_ + j]) {
          bound_v1[j] = points[i * d_ + j];
        }

        if (bound_v2[j] < points[i * d_ + j]) {
          bound_v1[j] = points[i * d_ + j];
        }
      }

    // Build the tree
    char *temp_node_data = node_data_;
    top_node_ = BuildNodes(points, 0, n - 1, &temp_node_data);

    // Cleanup
    PointFree(bound_v1);
    PointFree(bound_v2);
  }

  ~KmTree() {
    free(point_indices_);
    free(node_data_);
  };

  // Given k cluster centers, this runs a full k-means iterations, choosing the next set of
  // centers and returning the cost function for this set of centers. If assignment is not null,
  // it should be an array of size n that will be filled with the index of the cluster (0 - k-1)
  // that each data point is assigned to. The new center values will overwrite the old ones.
  Scalar DoKMeansStep(int k, Scalar *centers, int *assignment) const {
    Scalar *bad_center = PointAllocate(d_);
    KM_ASSERT(bad_center != 0);
    memset(bad_center, 0xff, d_ * sizeof(Scalar));

    // Allocate data
    Scalar *sums = (Scalar *)calloc(k * d_, sizeof(Scalar));
    int *counts = (int *)calloc(k, sizeof(int));
    int num_candidates = 0;
    int *candidates = (int *)malloc(k * sizeof(int));
    KM_ASSERT(sums != 0 && counts != 0 && candidates != 0);

    for (int i = 0; i < k; i++)
      if (memcmp(centers + i * d_, bad_center, d_ * sizeof(Scalar)) != 0) {
        candidates[num_candidates++] = i;
      }

    // Find nodes
    Scalar result = DoKMeansStepAtNode(top_node_, num_candidates, candidates, centers, sums,
                                       counts, assignment);

    // Set the new centers
    for (int i = 0; i < k; i++) {
      if (counts[i] > 0) {
        PointScale(sums + i * d_, Scalar(1) / counts[i], d_);
        PointCopy(centers + i * d_, sums + i * d_, d_);
      } else {
        memcpy(centers + i * d_, bad_center, d_ * sizeof(Scalar));
      }
    }

    // Cleanup memory
    PointFree(bad_center);
    free(candidates);
    free(counts);
    free(sums);
    return result;
  };

  // Choose k initial centers for k-means using the kmeans++ seeding procedure. The resulting
  // centers are returned via the centers variable, which should be pre-allocated to size k*d.
  // The cost of the initial clustering is returned.
  Scalar SeedKMeansPlusPlus(int k, Scalar *centers) const {
    Scalar *dist_sq = (Scalar *)malloc(n_ * sizeof(Scalar));
    KM_ASSERT(dist_sq != 0);

    // Choose an initial center uniformly at random
    SeedKmppSetClusterIndex(top_node_, 0);
    int i = GetRandom(n_);
    memcpy(centers, points_ + point_indices_[i]*d_, d_ * sizeof(Scalar));
    Scalar total_cost = 0;

    for (int j = 0; j < n_; j++) {
      dist_sq[j] = PointDistSq(points_ + point_indices_[j] * d_, centers, d_);
      total_cost += dist_sq[j];
    }

    // Repeatedly choose more centers
    for (int new_cluster = 1; new_cluster < k; new_cluster++) {
      while (1) {
        Scalar cutoff = (rand() / Scalar(RAND_MAX)) * total_cost;
        Scalar cur_cost = 0;

        for (i = 0; i < n_; i++) {
          cur_cost += dist_sq[i];

          if (cur_cost >= cutoff) {
            break;
          }
        }

        if (i < n_) {
          break;
        }
      }

      memcpy(centers + new_cluster * d_, points_ + point_indices_[i]*d_, d_ * sizeof(Scalar));
      total_cost = SeedKmppUpdateAssignment(top_node_, new_cluster, centers, dist_sq);
    }

    // Clean up and return
    free(dist_sq);
    return total_cost;
  };

 private:
  struct Node {
    int num_points;                 // Number of points stored in this node
    int first_point_index;          // The smallest point index stored in this node
    Scalar *median, *radius;        // Bounding box center and half side-lengths
    Scalar *sum;                    // Sum of the points stored in this node
    Scalar opt_cost;                // Min cost for putting all points in this node in 1 cluster
    Node *lower_node, *upper_node;  // Child nodes
    mutable int kmpp_cluster_index; // The cluster these points are assigned to or -1 if variable
  };

  // Helper functions for constructor
  Node *BuildNodes(Scalar *points, int first_index, int last_index, char **next_node_data) {
    // Allocate the node
    Node *node = (Node *)(*next_node_data);
    (*next_node_data) += sizeof(Node);
    node->sum = (Scalar *)(*next_node_data);
    (*next_node_data) += sizeof(Scalar) * d_;
    node->median = (Scalar *)(*next_node_data);
    (*next_node_data) += sizeof(Scalar) * d_;
    node->radius = (Scalar *)(*next_node_data);
    (*next_node_data) += sizeof(Scalar) * d_;

    // Fill in basic info
    node->num_points = (last_index - first_index + 1);
    node->first_point_index = first_index;

    // Calculate the bounding box
    Scalar *first_point = points + point_indices_[first_index] * d_;
    Scalar *bound_p1 = PointAllocate(d_);
    Scalar *bound_p2 = PointAllocate(d_);
    KM_ASSERT(bound_p1 != 0 && bound_p2 != 0);
    PointCopy(bound_p1, first_point, d_);
    PointCopy(bound_p2, first_point, d_);

    for (int i = first_index + 1; i <= last_index; i++)
      for (int j = 0; j < d_; j++) {
        Scalar c = points[point_indices_[i] * d_ + j];

        if (bound_p1[j] > c) {
          bound_p1[j] = c;
        }

        if (bound_p2[j] < c) {
          bound_p2[j] = c;
        }
      }

    // Calculate bounding box stats and delete the bounding box memory
    Scalar max_radius = -1;
    int split_d = -1;

    for (int j = 0; j < d_; j++) {
      node->median[j] = (bound_p1[j] + bound_p2[j]) / 2;
      node->radius[j] = (bound_p2[j] - bound_p1[j]) / 2;

      if (node->radius[j] > max_radius) {
        max_radius = node->radius[j];
        split_d = j;
      }
    }

    PointFree(bound_p2);
    PointFree(bound_p1);

    // If the max spread is 0, make this a leaf node
    if (max_radius == 0) {
      node->lower_node = node->upper_node = 0;
      PointCopy(node->sum, first_point, d_);

      if (last_index != first_index) {
        PointScale(node->sum, Scalar(last_index - first_index + 1), d_);
      }

      node->opt_cost = 0;
      return node;
    }

    // Partition the points around the midpoint in this dimension. The partitioning is done in-place
    // by iterating from left-to-right and right-to-left in the same way that partioning is done for
    // quicksort.
    Scalar split_pos = node->median[split_d];
    int i1 = first_index, i2 = last_index, size1 = 0;

    while (i1 <= i2) {
      bool is_i1_good = (points[point_indices_[i1] * d_ + split_d] < split_pos);
      bool is_i2_good = (points[point_indices_[i2] * d_ + split_d] >= split_pos);

      if (!is_i1_good && !is_i2_good) {
        int temp = point_indices_[i1];
        point_indices_[i1] = point_indices_[i2];
        point_indices_[i2] = temp;
        is_i1_good = is_i2_good = true;
      }

      if (is_i1_good) {
        i1++;
        size1++;
      }

      if (is_i2_good) {
        i2--;
      }
    }

    // Create the child nodes
    KM_ASSERT(size1 >= 1 && size1 <= last_index - first_index);
    node->lower_node = BuildNodes(points, first_index, first_index + size1 - 1, next_node_data);
    node->upper_node = BuildNodes(points, first_index + size1, last_index, next_node_data);

    // Calculate the new sum and opt cost
    PointCopy(node->sum, node->lower_node->sum, d_);
    PointAdd(node->sum, node->upper_node->sum, d_);
    Scalar *center = PointAllocate(d_);
    KM_ASSERT(center != 0);
    PointCopy(center, node->sum, d_);
    PointScale(center, Scalar(1) / node->num_points, d_);
    node->opt_cost = GetNodeCost(node->lower_node, center) + GetNodeCost(node->upper_node, center);
    PointFree(center);
    return node;
  };
  Scalar GetNodeCost(const Node *node, Scalar *center) const {
    Scalar dist_sq = 0;

    for (int i = 0; i < d_; i++) {
      Scalar x = (node->sum[i] / node->num_points) - center[i];
      dist_sq += x * x;
    }

    return node->opt_cost + node->num_points * dist_sq;
  };

  // Helper functions for DoKMeans step
  Scalar DoKMeansStepAtNode(const Node *node, int k, int *candidates, Scalar *centers,
                            Scalar *sums, int *counts, int *assignment) const {
    // Determine which center the node center is closest to
    Scalar min_dist_sq = PointDistSq(node->median, centers + candidates[0] * d_, d_);
    int closest_i = candidates[0];

    for (int i = 1; i < k; i++) {
      Scalar dist_sq = PointDistSq(node->median, centers + candidates[i] * d_, d_);

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_i = candidates[i];
      }
    }

    // If this is a non-leaf node, recurse if necessary
    if (node->lower_node != 0) {
      // Build the new list of candidates
      int new_k = 0;
      int *new_candidates = (int *)malloc(k * sizeof(int));
      KM_ASSERT(new_candidates != 0);

      for (int i = 0; i < k; i++)
        if (!ShouldBePruned(node->median, node->radius, centers, closest_i, candidates[i])) {
          new_candidates[new_k++] = candidates[i];
        }

      // Recurse if there's at least two
      if (new_k > 1) {
        Scalar result = DoKMeansStepAtNode(node->lower_node, new_k, new_candidates, centers,
                                           sums, counts, assignment) +
                        DoKMeansStepAtNode(node->upper_node, new_k, new_candidates, centers,
                                           sums, counts, assignment);
        free(new_candidates);
        return result;
      } else {
        free(new_candidates);
      }
    }

    // Assigns all points within this node to a single center
    PointAdd(sums + closest_i * d_, node->sum, d_);
    counts[closest_i] += node->num_points;

    if (assignment != 0) {
      for (int i = node->first_point_index; i < node->first_point_index + node->num_points; i++) {
        assignment[point_indices_[i]] = closest_i;
      }
    }

    return GetNodeCost(node, centers + closest_i * d_);
  };
  bool ShouldBePruned(Scalar *box_median, Scalar *box_radius, Scalar *centers, int best_index,
                      int test_index) const {
    if (best_index == test_index) {
      return false;
    }

    Scalar *best = centers + best_index * d_;
    Scalar *test = centers + test_index * d_;
    Scalar lhs = 0, rhs = 0;

    for (int i = 0; i < d_; i++) {
      Scalar component = test[i] - best[i];
      lhs += component * component;

      if (component > 0) {
        rhs += (box_median[i] + box_radius[i] - best[i]) * component;
      } else {
        rhs += (box_median[i] - box_radius[i] - best[i]) * component;
      }
    }

    return (lhs >= 2 * rhs);
  };

  // Helper functions for SeedKMeansPlusPlus
  void SeedKmppSetClusterIndex(const Node *node, int index) const {
    node->kmpp_cluster_index = index;

    if (node->lower_node != 0) {
      SeedKmppSetClusterIndex(node->lower_node, index);
      SeedKmppSetClusterIndex(node->upper_node, index);
    }
  };
  Scalar SeedKmppUpdateAssignment(const Node *node, int new_cluster, Scalar *centers,
                                  Scalar *dist_sq) const {
    // See if we can assign all points in this node to one cluster
    if (node->kmpp_cluster_index >= 0) {
      if (ShouldBePruned(node->median, node->radius, centers, node->kmpp_cluster_index, new_cluster)) {
        return GetNodeCost(node, centers + node->kmpp_cluster_index * d_);
      }

      if (ShouldBePruned(node->median, node->radius, centers, new_cluster,
                         node->kmpp_cluster_index)) {
        SeedKmppSetClusterIndex(node, new_cluster);

        for (int i = node->first_point_index; i < node->first_point_index + node->num_points; i++) {
          dist_sq[i] = PointDistSq(points_ + point_indices_[i] * d_, centers + new_cluster * d_, d_);
        }

        return GetNodeCost(node, centers + new_cluster * d_);
      }

      // It may be that the a leaf-node point is equidistant from the new center or old
      if (node->lower_node == 0) {
        return GetNodeCost(node, centers + node->kmpp_cluster_index * d_);
      }
    }

    // Recurse
    Scalar cost = SeedKmppUpdateAssignment(node->lower_node, new_cluster, centers, dist_sq) +
                  SeedKmppUpdateAssignment(node->upper_node, new_cluster, centers, dist_sq);
    int i1 = node->lower_node->kmpp_cluster_index, i2 = node->upper_node->kmpp_cluster_index;

    if (i1 == i2 && i1 != -1) {
      node->kmpp_cluster_index = i1;
    } else {
      node->kmpp_cluster_index = -1;
    }

    return cost;
  };

  int n_, d_;
  Scalar *points_;
  Node *top_node_;
  char *node_data_;
  int *point_indices_;
};


// Returns the number of seconds since the program began execution.
inline double GetSeconds() {
  return double(clock()) / CLOCKS_PER_SEC;
}

// See KMeans.h
// Performs one full execution of k-means, logging any relevant information, and tracking meta
// statistics for the run. If min or max values are negative, they are treated as unset.
// best_centers and best_assignment can be 0, in which case they are not set.
inline void RunKMeansOnce(const KmTree &tree, int n, int k, int d, Scalar *points, Scalar *centers,
                          Scalar *min_cost, Scalar *max_cost, Scalar *total_cost,
                          double start_time, double *min_time, double *max_time,
                          double *total_time, Scalar *best_centers, int *best_assignment) {
  const Scalar kEpsilon = Scalar(1e-8);  // Used to determine when to terminate k-means

  // Do iterations of k-means until the cost stabilizes
  Scalar old_cost = 0;
  bool is_done = false;

  for (int iteration = 0; !is_done; iteration++) {
    Scalar new_cost = tree.DoKMeansStep(k, centers, 0);
    is_done = (iteration > 0 && new_cost >= (1 - kEpsilon) * old_cost);
    old_cost = new_cost;
  }

  double this_time = GetSeconds() - start_time;

  // Log the clustering we found

  // Handle a new min cost, updating best_centers and best_assignment as appropriate
  if (*min_cost < 0 || old_cost < *min_cost) {
    *min_cost = old_cost;

    if (best_assignment != 0) {
      tree.DoKMeansStep(k, centers, best_assignment);
    }

    if (best_centers != 0) {
      memcpy(best_centers, centers, sizeof(Scalar)*k * d);
    }
  }

  // Update all other aggregate stats
  if (*max_cost < old_cost) {
    *max_cost = old_cost;
  }

  *total_cost += old_cost;

  if (*min_time < 0 || *min_time > this_time) {
    *min_time = this_time;
  }

  if (*max_time < this_time) {
    *max_time = this_time;
  }

  *total_time += this_time;
}


// Runs k-means on the given set of points.
//   - n: The number of points in the data set
//   - k: The number of clusters to look for
//   - d: The number of dimensions that the data set lives in
//   - points: An array of size n*d where points[d*i + j] gives coordinate j of point i
//   - attempts: The number of times to independently run k-means with different starting centers.
//               The best result is always returned (as measured by the cost function).
//   - centers: This can either be null or an array of size k*d. In the latter case, it will be
//              filled with the locations of all final cluster centers. Specifically
//              centers[d*i + j] will give coordinate j of center i. If the cluster is unused, it
//              will contain NaN instead.
//   - assignments: This can either be null or an array of size n. In the latter case, it will be
//                  filled with the cluster that each point is assigned to (an integer between 0
//                  and k-1 inclusive).
// The final cost of the clustering is also returned.
// See KMeans.h
Scalar RunKMeans(int n, int k, int d, Scalar *points, int attempts,
                 Scalar *ret_centers, int *ret_assignment) {
  KM_ASSERT(k >= 1);

  // Create the tree and log
  KmTree tree(n, d, points);

  // Initialization
  Scalar *centers = (Scalar *)malloc(sizeof(Scalar) * k * d);
  int *unused_centers = (int *)malloc(sizeof(int) * n);
  KM_ASSERT(centers != 0 && unused_centers != 0);
  Scalar min_cost = -1, max_cost = -1, total_cost = 0;
  double min_time = -1, max_time = -1, total_time = 0;

  // Handle k > n
  if (k > n) {
    memset(centers + n * d, -1, (k - d)*sizeof(Scalar));
    k = n;
  }

  // Run all the attempts
  for (int attempt = 0; attempt < attempts; attempt++) {
    double start_time = GetSeconds();

    // Choose centers uniformly at random
    for (int i = 0; i < n; i++) {
      unused_centers[i] = i;
    }

    int num_unused_centers = n;

    for (int i = 0; i < k; i++) {
      int j = GetRandom(num_unused_centers--);
      memcpy(centers + i * d, points + unused_centers[j]*d, d * sizeof(Scalar));
      unused_centers[j] = unused_centers[num_unused_centers];
    }

    // Run k-means
    RunKMeansOnce(tree, n, k, d, points, centers, &min_cost, &max_cost, &total_cost, start_time,
                  &min_time, &max_time, &total_time, ret_centers, ret_assignment);
  }

  // Clean up and return
  free(unused_centers);
  free(centers);
  return min_cost;
}

// See KMeans.h
Scalar RunKMeansPlusPlus(int n, int k, int d, Scalar *points, int attempts,
                         Scalar *ret_centers, int *ret_assignment) {
  KM_ASSERT(k >= 1);

  // Create the tree and log
  KmTree tree(n, d, points);

  // Initialization
  Scalar *centers = (Scalar *)malloc(sizeof(Scalar) * k * d);
  KM_ASSERT(centers != 0);
  Scalar min_cost = -1, max_cost = -1, total_cost = 0;
  double min_time = -1, max_time = -1, total_time = 0;

  // Run all the attempts
  for (int attempt = 0; attempt < attempts; attempt++) {
    double start_time = GetSeconds();

    // Choose centers using k-means++ seeding
    tree.SeedKMeansPlusPlus(k, centers);

    // Run k-means
    RunKMeansOnce(tree, n, k, d, points, centers, &min_cost, &max_cost, &total_cost, start_time,
                  &min_time, &max_time, &total_time, ret_centers, ret_assignment);
  }

  // Clean up and return
  free(centers);
  return min_cost;
}

namespace detail {
// Auxiliary method: templatized for working with float/double's.
template <typename SCALAR>
double  internal_kmeans(
  const bool use_kmeansplusplus_method,
  const size_t nPoints,
  const size_t k,
  const size_t dims,
  const SCALAR *points,
  const size_t attempts,
  SCALAR *out_center,
  int *out_assignments
);


template <>
double internal_kmeans<double>(
  const bool use_kmeansplusplus_method,
  const size_t nPoints,
  const size_t k,
  const size_t dims,
  const double *points,
  const size_t attempts,
  double *out_center,
  int *out_assignments) {
  return RunKMeans(nPoints, k, dims, const_cast<double *>(points), attempts, out_center,
                   out_assignments);
}


template <>
double internal_kmeans<float>(
  const bool use_kmeansplusplus_method,
  const size_t nPoints,
  const size_t k,
  const size_t dims,
  const float *points,
  const size_t attempts,
  float *out_center,
  int *out_assignments) {
  std::vector<double>  points_d(nPoints * dims);
  std::vector<double>  centers_d(k * dims);

  // Convert: float -> double
  for (size_t i = 0; i < nPoints * dims; i++) {
    points_d[i] = double(points[i]);
  }

  const double ret = RunKMeans(nPoints, k, dims, &points_d[0], attempts, &centers_d[0],
                               out_assignments);

  // Convert: double -> float
  if (out_center)
    for (size_t i = 0; i < k * dims; i++) {
      out_center[i] = float(centers_d[i]);
    }

  return ret;
}

// Auxiliary method, the actual code of the two front-end functions offered to the user below.
template <class LIST_OF_VECTORS1, class LIST_OF_VECTORS2>
double stub_kmeans(
  const bool use_kmeansplusplus_method,
  const size_t k,
  const LIST_OF_VECTORS1 &points,
  std::vector<int>  &assignments,
  LIST_OF_VECTORS2 *out_centers,
  const size_t attempts
) {
  assert(k >= 1);

  const size_t N = points.size();
  assignments.resize(N);

  if (out_centers) {
    out_centers->clear();
  }

  if (!N) {
    return 0;  // No points, we're done.
  }

  // Parse to required format:
  size_t dims = 0;
  const typename LIST_OF_VECTORS1::const_iterator it_first = points.begin();
  const typename LIST_OF_VECTORS1::const_iterator it_end  = points.end();
  typedef typename LIST_OF_VECTORS1::value_type TInnerVector;
  typedef typename LIST_OF_VECTORS2::value_type TInnerVectorCenters;

  std::vector<typename TInnerVector::value_type> raw_vals;
  typename TInnerVector::value_type *trg_ptr = NULL;

  for (typename LIST_OF_VECTORS1::const_iterator it = it_first; it != it_end; ++it) {
    if (it == it_first) {
      dims = it->size();
      assert(dims > 0);
      raw_vals.resize(N * dims);
      trg_ptr = &raw_vals[0];
    } else {
      assert(size_t(dims) == size_t(it->size()));
    }

    ::memcpy(trg_ptr, &(*it)[0], dims * sizeof(typename TInnerVector::value_type));
    trg_ptr += dims;
  }

  // Call the internal implementation:
  std::vector<typename TInnerVectorCenters::value_type> centers(dims * k);
  const double ret = detail::internal_kmeans(false, N, k, points.begin()->size(), &raw_vals[0],
                     attempts, &centers[0], &assignments[0]);

  // Centers:
  if (out_centers) {
    const typename TInnerVectorCenters::value_type *center_ptr = &centers[0];

    for (size_t i = 0; i < k; i++) {
      TInnerVectorCenters c;

      for (size_t j = 0; j < dims; j++) {
        if (j < c.size()) {
          c[j] = *center_ptr++;
        }
      }

      out_centers->push_back(c);
    }
  }

  return ret;
}
} // end detail namespace


/** @name k-means algorithms
@{ */

/** k-means algorithm to cluster a list of N points of arbitrary dimensionality into exactly K clusters.
  *   The list of input points can be any template CONTAINER<POINT> with:
  *		- CONTAINER can be: Any STL container: std::vector,std::list,std::deque,...
  *		- POINT can be:
  *			- std::vector<double/float>
  *			- CArrayDouble<N> / CArrayFloat<N>
  *
  *  \param k [IN] Number of cluster to look for.
  *  \param points [IN] The list of N input points. It can be any STL-like containers of std::vector<float/double>, for example a std::vector<mrpt::math::CVectorDouble>, a std::list<CVectorFloat>, etc...
  *  \param assignments [OUT] At output it will have a number [0,k-1] for each of the N input points.
  *  \param out_centers [OUT] If not NULL, at output will have the centers of each group. Can be of any of the supported types of "points", but the basic coordinates should be float or double exactly as in "points".
  *  \param attempts [IN] Number of attempts.
  *
  * \sa A more advanced algorithm, see: kmeanspp
  * \note Uses the kmeans++ implementation by David Arthur (2009, http://www.stanford.edu/~darthur/kmpp.zip).
  */
template <class LIST_OF_VECTORS1, class LIST_OF_VECTORS2>
inline double kmeans(
  const size_t k,
  const LIST_OF_VECTORS1 &points,
  std::vector<int>  &assignments,
  LIST_OF_VECTORS2 *out_centers = NULL,
  const size_t attempts = 3
) {
  return detail::stub_kmeans(false /* standard method */, k, points, assignments, out_centers,
                             attempts);
}

/** k-means++ algorithm to cluster a list of N points of arbitrary dimensionality into exactly K clusters.
  *   The list of input points can be any template CONTAINER<POINT> with:
  *		- CONTAINER can be: Any STL container: std::vector,std::list,std::deque,...
  *		- POINT can be:
  *			- std::vector<double/float>
  *			- CArrayDouble<N> / CArrayFloat<N>
  *
  *  \param k [IN] Number of cluster to look for.
  *  \param points [IN] The list of N input points. It can be any STL-like containers of std::vector<float/double>, for example a std::vector<mrpt::math::CVectorDouble>, a std::list<CVectorFloat>, etc...
  *  \param assignments [OUT] At output it will have a number [0,k-1] for each of the N input points.
  *  \param out_centers [OUT] If not NULL, at output will have the centers of each group. Can be of any of the supported types of "points", but the basic coordinates should be float or double exactly as in "points".
  *  \param attempts [IN] Number of attempts.
  *
  * \sa The standard kmeans algorithm, see: kmeans
  * \note Uses the kmeans++ implementation by David Arthur (2009, http://www.stanford.edu/~darthur/kmpp.zip).
  */
template <class LIST_OF_VECTORS1, class LIST_OF_VECTORS2>
inline double kmeanspp(
  const size_t k,
  const LIST_OF_VECTORS1 &points,
  std::vector<int>  &assignments,
  LIST_OF_VECTORS2 *out_centers = NULL,
  const size_t attempts = 3
) {
  return detail::stub_kmeans(true /* kmeans++ algorithm*/, k, points, assignments, out_centers,
                             attempts);
}

/** @} */

} // End of MATH namespace
} // End of namespace
#endif
