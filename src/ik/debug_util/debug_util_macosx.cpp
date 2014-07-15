#ifdef __APPLE__

#include <stdio.h>  // printf
#include "ik/debug_util/debug_util.h"

namespace ik {
namespace debug {

  void EnableMemoryLeakChecks() {
    printf("WARNING: EnableMemoryLeakChecks not implemented yet for Mac\n");
  }

  void SetBreakPointOnAlocation(int alloc_num) {
    static_cast<void>(alloc_num);
    printf("WARNING: SetBreakPointOnAlocation not implemented yet for Mac\n");
  }

}  // namespace debug
}  // namespace ik

#endif  // __APPLE__
