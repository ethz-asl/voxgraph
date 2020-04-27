#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_

#include <algorithm>
#include <random>
#include <vector>

namespace voxgraph {
template <typename ItemType>
class WeightedSampler {
 public:
  using ItemContainer = std::vector<ItemType>;

  WeightedSampler() = default;

  void addItem(const ItemType& new_item, const double weight);

  // Deterministically get the i-th item
  inline const ItemType& operator[](int i) const { return items_[i]; }

  // Randomly draw an item with probability proportional to its weight
  inline const ItemType& getRandomItem() const;

  size_t size() const { return items_.size(); }

  void clear();

 private:
  // Items and vector storing their cumulative probabilities
  ItemContainer items_;
  std::vector<double> cumulative_item_weights_;

  // Uniform random number generator
  mutable std::mt19937 random_number_generator_;
  mutable std::uniform_real_distribution<double> uniform_distribution_{0.0,
                                                                       1.0};
};
}  // namespace voxgraph

#include "voxgraph/frontend/submap_collection/weighted_sampler_inl.h"

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_
