#ifndef __VMMEM_H__
#define __VMMEM_H__

#include <stdlib.h>
#include <string>
#include <vector>

#define VMMEM_READ   4
#define VMMEM_WRITE  2
#define VMMEM_EXEC   1

using VmHandle = int;
using VmPerm = std::vector<std::pair<VmHandle, uint32_t>>;

class VmMem {
 public:
  virtual ~VmMem() = 0;
  static std::unique_ptr<VmMem> CreateVmMem();
  virtual int FindVmByName(const std::string& vm_name);
  virtual int LendDmabuf(int fd, const VmPerm& vm_perms);
};

#endif  // __VMMEM_H__
