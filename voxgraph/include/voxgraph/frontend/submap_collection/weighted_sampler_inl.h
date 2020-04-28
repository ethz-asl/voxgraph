#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_INL_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_INL_H_

namespace voxgraph {
template <typename ItemType>
void WeightedSampler<ItemType>::addItem(const ItemType& new_item,
                                        const double weight) {
  items_.push_back(new_item);
  if (cumulative_item_weights_.empty()) {
    cumulative_item_weights_.push_back(weight);
  } else {
    const double new_item_cumulative_weight =
        cumulative_item_weights_.back() + weight;
    cumulative_item_weights_.push_back(new_item_cumulative_weight);
  }
}

template <typename ItemType>
const ItemType& WeightedSampler<ItemType>::getRandomItem() const {
  const double random_number = uniform_distribution_(random_number_generator_);
  const double random_cumulative_weight =
      random_number * cumulative_item_weights_.back();
  const auto it = std::upper_bound(cumulative_item_weights_.begin(),
                                   cumulative_item_weights_.end(),
                                   random_cumulative_weight);
  const unsigned int random_index = it - cumulative_item_weights_.begin();
  return items_[random_index];
}

template <typename ItemType>
void WeightedSampler<ItemType>::clear() {
  items_.clear();
  cumulative_item_weights_.clear();
}
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_WEIGHTED_SAMPLER_INL_H_
