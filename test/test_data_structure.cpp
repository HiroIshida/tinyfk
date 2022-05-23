#include <cassert>
#include <data_structure.hpp>

int main() {
  tinyfk::SizedStack<int> s(100);
  s.top();
  assert(s.empty());
  s.push(1);
  s.push(2);
  assert(s.size() == 2);
  assert(s.top() == 2);
  s.pop();
  s.pop();
  assert(s.empty());
  s.reset();
  s.push(100);
  s.push(200);
  assert(s.top() == 200);
}
