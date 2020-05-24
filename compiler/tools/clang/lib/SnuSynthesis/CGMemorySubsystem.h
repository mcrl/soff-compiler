/*****************************************************************************/
/*                                                                           */
/* Copyright (c) 2020 Seoul National University.                             */
/* All rights reserved.                                                      */
/*                                                                           */
/* Redistribution and use in source and binary forms, with or without        */
/* modification, are permitted provided that the following conditions        */
/* are met:                                                                  */
/*   1. Redistributions of source code must retain the above copyright       */
/*      notice, this list of conditions and the following disclaimer.        */
/*   2. Redistributions in binary form must reproduce the above copyright    */
/*      notice, this list of conditions and the following disclaimer in the  */
/*      documentation and/or other materials provided with the distribution. */
/*   3. Neither the name of Seoul National University nor the names of its   */
/*      contributors may be used to endorse or promote products derived      */
/*      from this software without specific prior written permission.        */
/*                                                                           */
/* THIS SOFTWARE IS PROVIDED BY SEOUL NATIONAL UNIVERSITY "AS IS" AND ANY    */
/* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE    */
/* DISCLAIMED. IN NO EVENT SHALL SEOUL NATIONAL UNIVERSITY BE LIABLE FOR ANY */
/* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL        */
/* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS   */
/* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)     */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       */
/* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  */
/* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/* POSSIBILITY OF SUCH DAMAGE.                                               */
/*                                                                           */
/* Contact information:                                                      */
/*   Center for Manycore Programming                                         */
/*   Department of Computer Science and Engineering                          */
/*   Seoul National University, Seoul 08826, Korea                           */
/*   http://aces.snu.ac.kr                                                   */
/*                                                                           */
/* Contributors:                                                             */
/*   Gangwon Jo, Heehoon Kim, Jeesoo Lee, and Jaejin Lee                     */
/*                                                                           */
/*****************************************************************************/

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CGMEMORYSUBSYSTEM_H
#define LLVM_CLANG_SNU_SYNTHESIS_CGMEMORYSUBSYSTEM_H

#include "CGCommon.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include <map>
#include <set>
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

class AXIInterface;

// Avalon Memory-Mapped Interface

class AMMInterfaceBase {
public:
  enum PortKind {
    AMMI_PK_None,
    AMMI_PK_MasterPort,
    AMMI_PK_SlavePort
  };

private:
  ModuleBuilder *Parent;
  std::string Name;
  PortKind PKind;
  VerilogSignalPortKind MtoS;
  VerilogSignalPortKind StoM;

  VerilogSignal *Address;
  VerilogSignal *Read;
  VerilogSignal *ReadData;
  VerilogSignal *ReadDataValid;
  VerilogSignal *Write;
  VerilogSignal *WriteData;
  VerilogSignal *ByteEnable;
  VerilogSignal *BurstCount;
  VerilogSignal *WaitRequest;
  VerilogSignal *WaitResponse;
  unsigned AddrWidth;
  unsigned DataWidth;
  unsigned BurstWidth;
  unsigned AddressSpace;
  unsigned AliasGroup;
  bool IsAtomic;

protected:
  AMMInterfaceBase(ModuleBuilder *parent, StringRef name, unsigned addrWidth,
                   unsigned dataWidth, unsigned burstWidth, PortKind portKind,
                   bool Readable, bool Writable, bool HasWaitResponse);

public:
  ModuleBuilder *parent() const { return Parent; }
  VerilogSignal *address() const { return Address; }
  VerilogSignal *read() const { return Read; }
  VerilogSignal *readdata() const { return ReadData; }
  VerilogSignal *readdatavalid() const { return ReadDataValid; }
  VerilogSignal *write() const { return Write; }
  VerilogSignal *writedata() const { return WriteData; }
  VerilogSignal *byteenable() const { return ByteEnable; }
  VerilogSignal *burstcount() const { return BurstCount; }
  VerilogSignal *waitrequest() const { return WaitRequest; }
  VerilogSignal *waitresponse() const { return WaitResponse; }
  StringRef getName() const { return Name; }
  std::string getNameAsString() const { return Name; }
  PortKind getPortKind() const { return PKind; }
  unsigned getAddressWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }
  unsigned getAlignWidth() const { return Log2(DataWidth / 8); }
  unsigned getBurstWidth() const { return BurstWidth; }

  bool isReadable() const { return Read; }
  bool isWritable() const { return Write; }
  bool isReadWrite() const { return Read && Write; }
  bool isReadOnly() const { return Read && !Write; }
  bool isWriteOnly() const { return !Read && Write; }
  bool isBurstAccessable() const { return BurstCount; }
  bool isHandshakable() const { return WaitResponse; }

  unsigned getAddressSpace() const { return AddressSpace; }
  unsigned getAliasGroup() const { return AliasGroup; }
  bool isAtomic() const { return IsAtomic; }

  void AnnotateAddressSpace(unsigned AS) {
    AddressSpace = AS;
  }
  void AnnotateAliasGroup(unsigned AG) {
    AliasGroup = AG;
  }
  void AnnotateAsAtomic() {
    IsAtomic = true;
  }

  void ConnectToSlave(AMMInterfaceBase *Slave);
  void ConnectToModuleInstanceAsMaster(VerilogModuleInstance *Slave,
                                       StringRef Prefix);
  void ConnectToModuleInstanceAsSlave(VerilogModuleInstance *Master,
                                      StringRef Prefix);
  void ConnectToAXISlave(AXIInterface *Slave);

  AMMInterfaceBase *Clone(ModuleBuilder *Parent, StringRef Name,
                          PortKind portKind = AMMI_PK_None) const;
  AMMInterfaceBase *ResizedClone(ModuleBuilder *Parent, StringRef Name,
                                 unsigned NewDataWidth,
                                 PortKind portKind = AMMI_PK_None) const;
};

class AMMInterface : public AMMInterfaceBase {
public:
  AMMInterface(ModuleBuilder *parent, StringRef Name, unsigned AddrWidth,
               unsigned DataWidth, unsigned BurstWidth = 0,
               PortKind portKind = AMMI_PK_None)
    : AMMInterfaceBase(parent, Name, AddrWidth, DataWidth, BurstWidth, portKind,
                       true, true, false) {}
};

class AMMReadOnlyInterface : public AMMInterfaceBase {
public:
  AMMReadOnlyInterface(ModuleBuilder *parent, StringRef Name,
                       unsigned AddrWidth, unsigned DataWidth,
                       unsigned BurstWidth = 0,
                       PortKind portKind = AMMI_PK_None)
    : AMMInterfaceBase(parent, Name, AddrWidth, DataWidth, BurstWidth, portKind,
                       true, false, false) {}
};

class AMMWriteOnlyInterface : public AMMInterfaceBase {
public:
  AMMWriteOnlyInterface(ModuleBuilder *parent, StringRef Name,
                        unsigned AddrWidth, unsigned DataWidth,
                        unsigned BurstWidth = 0,
                        PortKind portKind = AMMI_PK_None)
    : AMMInterfaceBase(parent, Name, AddrWidth, DataWidth, BurstWidth, portKind,
                       false, true, false) {}
};

class HAMMInterface : public AMMInterfaceBase {
public:
  HAMMInterface(ModuleBuilder *parent, StringRef Name, unsigned AddrWidth,
                unsigned DataWidth, unsigned BurstWidth = 0,
                PortKind portKind = AMMI_PK_None)
    : AMMInterfaceBase(parent, Name, AddrWidth, DataWidth, BurstWidth, portKind,
                       true, true, true) {}
};

class HAMMReadOnlyInterface : public AMMInterfaceBase {
public:
  HAMMReadOnlyInterface(ModuleBuilder *parent, StringRef Name,
                        unsigned AddrWidth, unsigned DataWidth,
                        unsigned BurstWidth = 0,
                        PortKind portKind = AMMI_PK_None)
    : AMMInterfaceBase(parent, Name, AddrWidth, DataWidth, BurstWidth, portKind,
                       true, false, true) {}
};

// AXI Interface

class AXIInterface {
public:
  enum PortKind {
    AXII_PK_None,
    AXII_PK_MasterPort,
    AXII_PK_SlavePort
  };

private:
  ModuleBuilder *Parent;
  std::string Name;
  PortKind PKind;
  VerilogSignal *ARAddr;
  VerilogSignal *ARBurst;
  VerilogSignal *ARCache;
  VerilogSignal *ARLen;
  VerilogSignal *ARReady;
  VerilogSignal *ARSize;
  VerilogSignal *ARValid;
  VerilogSignal *AWAddr;
  VerilogSignal *AWBurst;
  VerilogSignal *AWCache;
  VerilogSignal *AWLen;
  VerilogSignal *AWReady;
  VerilogSignal *AWSize;
  VerilogSignal *AWValid;
  VerilogSignal *BReady;
  VerilogSignal *BResp;
  VerilogSignal *BValid;
  VerilogSignal *RData;
  VerilogSignal *RLast;
  VerilogSignal *RReady;
  VerilogSignal *RResp;
  VerilogSignal *RValid;
  VerilogSignal *WData;
  VerilogSignal *WLast;
  VerilogSignal *WReady;
  VerilogSignal *WStrb;
  VerilogSignal *WValid;
  unsigned AddrWidth;
  unsigned DataWidth;
  unsigned AddressSpace;
  unsigned AliasGroup;

public:
  AXIInterface(ModuleBuilder *parent, StringRef name, unsigned addrWidth,
               unsigned dataWidth, PortKind portKind = AXII_PK_None);

  ModuleBuilder *parent() const { return Parent; }
  VerilogSignal *araddr() const { return ARAddr; }
  VerilogSignal *arburst() const { return ARBurst; }
  VerilogSignal *arcache() const { return ARCache; }
  VerilogSignal *arlen() const { return ARLen; }
  VerilogSignal *arready() const { return ARReady; }
  VerilogSignal *arsize() const { return ARSize; }
  VerilogSignal *arvalid() const { return ARValid; }
  VerilogSignal *awaddr() const { return AWAddr; }
  VerilogSignal *awburst() const { return AWBurst; }
  VerilogSignal *awcache() const { return AWCache; }
  VerilogSignal *awlen() const { return AWLen; }
  VerilogSignal *awready() const { return AWReady; }
  VerilogSignal *awsize() const { return AWSize; }
  VerilogSignal *awvalid() const { return AWValid; }
  VerilogSignal *bready() const { return BReady; }
  VerilogSignal *bresp() const { return BResp; }
  VerilogSignal *bvalid() const { return BValid; }
  VerilogSignal *rdata() const { return RData; }
  VerilogSignal *rlast() const { return RLast; }
  VerilogSignal *rready() const { return RReady; }
  VerilogSignal *rresp() const { return RResp; }
  VerilogSignal *rvalid() const { return RValid; }
  VerilogSignal *wdata() const { return WData; }
  VerilogSignal *wlast() const { return WLast; }
  VerilogSignal *wready() const { return WReady; }
  VerilogSignal *wstrb() const { return WStrb; }
  VerilogSignal *wvalid() const { return WValid; }
  StringRef getName() const { return Name; }
  std::string getNameAsString() const { return Name; }
  PortKind getPortKind() const { return PKind; }
  unsigned getAddressWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  unsigned getAddressSpace() const { return AddressSpace; }
  unsigned getAliasGroup() const { return AliasGroup; }

  void AnnotateAddressSpace(unsigned AS) {
    AddressSpace = AS;
  }
  void AnnotateAliasGroup(unsigned AG) {
    AliasGroup = AG;
  }

  void ConnectToSlave(AXIInterface *Slave);
  void ConnectToModuleInstance(VerilogModuleInstance *Inst, StringRef Prefix);

  AXIInterface *Clone(ModuleBuilder *Parent, StringRef Name,
                      PortKind portKind = AXII_PK_None) const;
};

// Global Memory Subsystem

class GlobalMemorySubsystem {
  ModuleBuilder *Parent;

  /* | 63 ... 32 | 31 ... 16 | 15 ... 0 |
   * |       0x0 | Datapath  |  Alias   | General-purpose caches
   * |       0x0 | Datapath  |  Alias   | Bypasses
   */
  union InterfaceGroupID {
    uint64_t Raw;
    struct {
      struct {
        uint16_t AliasGroup;
        uint16_t Datapath;
      } PerBufferIndex;
      uint32_t Reserved;
    } Parts;

    InterfaceGroupID() { Raw = 0; }
    InterfaceGroupID(uint64_t raw) { Raw = raw; }

    bool operator==(const InterfaceGroupID &RHS) const {
      return Raw == RHS.Raw;
    }
    bool operator!=(const InterfaceGroupID &RHS) const {
      return Raw != RHS.Raw;
    }
  };

  typedef SmallVector<AMMInterfaceBase*, 16> InterfaceList;
  typedef std::map<uint64_t, InterfaceList> GroupInterfaceListMap;
  GroupInterfaceListMap Interfaces;
  std::set<uint64_t> AtomicGroups;

  SmallVector<AMMInterface*, 64> AMMPorts;
  SmallVector<AXIInterface*, 64> AXIPorts;
  unsigned NumConsumedPorts;
  SmallVector<VerilogSignal*, 64> SubCleaned;

  bool IsFinalized;

public:
  explicit GlobalMemorySubsystem(ModuleBuilder *parent);

  void addInterface(unsigned Datapath, AMMInterfaceBase *I);
  void addGlobalMemoryPort(AMMInterface *M);
  void addGlobalMemoryPort(AXIInterface *M);

  void Finalize(VerilogSignal *Clean, VerilogSignal *Cleaned);

private:
  void SynthesizeArbiter(InterfaceList &L, unsigned NumSlaves = 0);

  unsigned ComputeNumRequiredPorts();
  void SynthesizeMoreAMMPorts(unsigned Required);
  void SynthesizeMoreAXIPorts(unsigned Required);
  void ConvertAXItoAMMPorts();

  void SynthesizeCaches(const InterfaceList &L, VerilogSignal *Clean);
  void SynthesizeBypass(AMMInterfaceBase *Master, VerilogSignal *Clean);
  void SynthesizeCache(AMMInterfaceBase *Master, VerilogSignal *Clean);

  void SynthesizeCleaned(VerilogSignal *GlobalCleaned);
};

// Local Memory Subsystem

class LocalMemoryLayout {
  CodeGenContext &Ctx;
  const ControlDataflowGraph *Program;
  const AliasSet &Aliases;

  unsigned NumLayers;
  unsigned LayerWidth;
  llvm::DenseMap<unsigned, uint64_t> Size;
  llvm::DenseMap<unsigned, unsigned> Granularity;
  llvm::DenseMap<WVarDecl*, uint64_t> Offset;

public:
  LocalMemoryLayout(CodeGenContext &ctx, const ControlDataflowGraph *program,
                    const AliasSet &aliases);

private:
  void Allocate(WVarDecl *Var, uint64_t AllocSize = 0);
  void UpdateGranularity(unsigned Alias, QualType AccessType);

public:
  uint64_t getAddressOf(WVarDecl *Var) const;
  unsigned getVirtualAddressWidthOf(unsigned Alias) const;
  unsigned getPhysicalAddressWidthOf(unsigned Alias) const;
  unsigned getDataWidthOf(unsigned Alias) const;
  unsigned getVirtualAddressWidthFor(DFGMemoryAccessNode *Access) const;
  unsigned getPhysicalAddressWidthFor(DFGMemoryAccessNode *Access) const;
  unsigned getDataWidthFor(DFGMemoryAccessNode *Access) const;
};

class LocalMemorySubsystem {
  ModuleBuilder *Parent;
  const LocalMemoryLayout &Layout;

  typedef SmallVector<AMMInterfaceBase*, 16> InterfaceList;
  typedef std::map<unsigned, InterfaceList> GroupInterfaceListMap;
  GroupInterfaceListMap Interfaces;

  bool IsFinalized;

public:
  LocalMemorySubsystem(ModuleBuilder *parent, const LocalMemoryLayout &layout);

  void addInterface(AMMInterfaceBase *I);

  void Finalize();

private:
  void SynthesizeLocalMemory(unsigned Alias, const InterfaceList &L);
};

} // namespace Synthesis

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CGMEMORYSUBSYSTEM_H
