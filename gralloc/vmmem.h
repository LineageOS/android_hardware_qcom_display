#ifndef __VMMEM_H__
#define __VMMEM_H__

#include <stdlib.h>
#include <string>
#include <vector>

#define VMMEM_READ   4
#define VMMEM_WRITE  2
#define VMMEM_EXEC   1

using VmPerm = std::vector<std::pair<int, unsigned int>>;
using VmHandle = int;

class VmMem {
 public:
  virtual ~VmMem() = 0;
  static std::unique_ptr<VmMem> CreateVmMem();
  virtual int FindVmByName(std::basic_string<char> vm_name);
  virtual int LendDmabuf(int fd, std::vector<std::pair<int, unsigned int>> vm_perms);
};

#endif  // __VMMEM_H__
