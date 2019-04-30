//
// Created by victor on 30.04.19.
//

#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_

#include <random>
#include <vector>

namespace voxgraph {
template <typename ItemType>
class WeightedSampler {
 public:
  using ItemContainer = std::vector<ItemType>;

  WeightedSampler() = default;

  void addItem(const ItemType &new_item, const double &weight) {
    items_.push_back(new_item);
    if (cumulative_item_weights_.empty()) {
      cumulative_item_weights_.push_back(weight);
    } else {
      const double new_item_cumulative_weight =
          cumulative_item_weights_.back() + weight;
      cumulative_item_weights_.push_back(new_item_cumulative_weight);
    }
  }

  // Deterministically get the i-th item
  inline const ItemType &operator[](int i) const { return items_[i]; }

  // Randomly draw an item with probability proportional to its weight
  inline const ItemType &getRandomItem() const {
    const double random_number =
        uniform_distribution_(random_number_generator_);
    const double random_cumulative_weight =
        random_number * cumulative_item_weights_.back();
    const auto it = std::upper_bound(cumulative_item_weights_.begin(),
                                     cumulative_item_weights_.end(),
                                     random_cumulative_weight);
    const unsigned int random_index = it - cumulative_item_weights_.begin();
    return items_[random_index];
  }

  size_t size() const { return items_.size(); }

  void clear() {
    items_.clear();
    cumulative_item_weights_.clear();
  }

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

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_H_
