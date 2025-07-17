#include "fake_nodes.hpp"

// 이 매크로는 한 번만, .cpp 파일에서 사용해야 합니다 :contentReference[oaicite:2]{index=2}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<FakePerception>("StrawberryPerception");
  factory.registerNodeType<FakeApproach>   ("RoughApproach");
  factory.registerNodeType<FakeServoCut>   ("ServoAndCut");
}
