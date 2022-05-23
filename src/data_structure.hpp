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
  assert(!isCachedVec_[id] && "attempt to break an existing cache");
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

template <class DataT> struct SizedStack {
  std::vector<DataT> tf_stack_;
  std::vector<size_t> hid_stack_; // here id stack
  SizedStack(){};
  SizedStack(size_t N_link)
      : tf_stack_(std::vector<DataT>(N_link)),
        hid_stack_(std::vector<size_t>(N_link)) {}
};

} // namespace tinyfk
