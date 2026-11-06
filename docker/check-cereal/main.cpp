#include <cereal/archives/binary.hpp>
#include <sstream>

int main()
{
  std::stringstream buffer;
  const int original = 42;
  {
    cereal::BinaryOutputArchive output(buffer);
    output(original);
  }
  int restored = 0;
  {
    cereal::BinaryInputArchive input(buffer);
    input(restored);
  }
  return restored == original ? 0 : 1;
}
