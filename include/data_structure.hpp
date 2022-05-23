#include <vector>

namespace tinyfk {

template <class DataT> class SizedCache {
public:
  explicit SizedCache(size_t cache_size)
      : cache_size_(cache_size), data_(std::vector<DataT>(cache_size)),
        cache_predicate_vector_(std::vector<bool>(cache_size, false)) {}

  SizedCache() : SizedCache(0) {}
  void set_cache(size_t id, const DataT &data);
  DataT *get_cache(size_t id);
  void extend();
  void clear();

private:
  int cache_size_;
  std::vector<DataT> data_;
  std::vector<bool> cache_predicate_vector_;
};

template <class DataT>
void SizedCache<DataT>::set_cache(size_t id, const DataT &tf) {
  // assert(!cache_predicate_vector_[id] && "attempt to break an existing
  // cache");
  cache_predicate_vector_[id] = true;
  data_[id] = tf;
}

template <class DataT> DataT *SizedCache<DataT>::get_cache(size_t id) {
  bool isAlreadyCached = (cache_predicate_vector_[id] == true);
  if (!isAlreadyCached) {
    return nullptr;
  } // the cache does not exists
  return &data_[id];
}

template <class DataT> void SizedCache<DataT>::extend() {
  cache_size_++;
  data_.push_back(DataT());
  cache_predicate_vector_.push_back(false);
  this->clear();
}

template <class DataT> void SizedCache<DataT>::clear() { // performance critical
  // bool's default value is false.
  cache_predicate_vector_ = std::vector<bool>(cache_size_);
}

template <class ElementT> class SizedStack {
public:
  SizedStack() : SizedStack(0) {}
  SizedStack(size_t max_stack_size)
      : data_(std::vector<ElementT>(max_stack_size)), current_idx_(0) {}

  inline size_t size() { return current_idx_; }
  inline bool empty() { return current_idx_ == 0; }
  inline void reset() { current_idx_ = 0; }
  inline void push(const ElementT &elem) {
    data_[current_idx_] = elem;
    current_idx_++;
  }
  inline ElementT &top() { return data_[current_idx_ - 1]; }
  inline void pop() { current_idx_--; }

private:
  size_t current_idx_;
  std::vector<ElementT> data_;
};

} // namespace tinyfk
