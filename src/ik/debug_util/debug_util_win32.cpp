#if defined(WIN32) || defined(_WIN32)

#include "ik/debug_util/debug_util.h"

namespace ik {
namespace debug {

  void EnableMemoryLeakChecks() {
#ifdef _DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
  }

  void EnableAggressiveMemoryLeakChecks() {
#ifdef _DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | 
      _CRTDBG_CHECK_ALWAYS_DF );
#endif
  }

  void SetBreakPointOnAlocation(int alloc_num) {
#ifdef _DEBUG
    _CrtSetBreakAlloc(alloc_num);
#endif
  }

}  // namespace debug
}  // namespace ik

#endif  // WIN32 || _WIN32
