#ifndef DYNAMIC_MAPPING_HELPERS_IMPL_H_
#define DYNAMIC_MAPPING_HELPERS_IMPL_H_
#include <unordered_set>

template <typename T>
std::vector<T> getUnique(const std::vector<T>& vec) {
  std::unordered_set<T> s;
  std::vector<T> res;
  for (const T& e : vec) {
    s.insert(e);
  }
  res.assign(s.begin(), s.end());
  std::sort(res.begin(), res.end());
  return res;
}

template <typename T>
std::unordered_map<T, int> getOccurencePerElement(const std::vector<T>& vec) {
  std::unordered_map<T, int> occurences;
  for (const T& el : vec) {
    if (occurences.find(el) == occurences.end()) {
      occurences.insert({el, 0});
    }
    occurences[el]++;
  }
  return occurences;
}
#endif