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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CGLOCKSUBSYSTEM_H
#define LLVM_CLANG_SNU_SYNTHESIS_CGLOCKSUBSYSTEM_H

#include "CGCommon.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

class BMLInterface {
public:
  enum PortKind {
    BMLI_PK_None,
    BMLI_PK_MasterPort,
    BMLI_PK_SlavePort
  };

private:
  ModuleBuilder *Parent;
  std::string Name;
  PortKind PKind;
  VerilogSignal *LockID;
  VerilogSignal *LockRequestValid;
  VerilogSignal *LockWaitRequest;
  VerilogSignal *LockResponseValid;
  VerilogSignal *LockWaitResponse;
  VerilogSignal *UnlockID;
  VerilogSignal *UnlockRequestValid;
  VerilogSignal *UnlockWaitRequest;
  VerilogSignal *UnlockResponseValid;
  VerilogSignal *UnlockWaitResponse;
  unsigned LockWidth;
  uint64_t Group;

public:
  BMLInterface(ModuleBuilder *parent, StringRef name, unsigned lockWidth,
               PortKind portKind = BMLI_PK_None);

public:
  ModuleBuilder *parent() const { return Parent; }
  VerilogSignal *lockid() const { return LockID; }
  VerilogSignal *lockrequestvalid() const { return LockRequestValid; }
  VerilogSignal *lockwaitrequest() const { return LockWaitRequest; }
  VerilogSignal *lockresponsevalid() const { return LockResponseValid; }
  VerilogSignal *lockwaitresponse() const { return LockWaitResponse; }
  VerilogSignal *unlockid() const { return UnlockID; }
  VerilogSignal *unlockrequestvalid() const { return UnlockRequestValid; }
  VerilogSignal *unlockwaitrequest() const { return UnlockWaitRequest; }
  VerilogSignal *unlockresponsevalid() const { return UnlockResponseValid; }
  VerilogSignal *unlockwaitresponse() const { return UnlockWaitResponse; }
  StringRef getName() const { return Name; }
  PortKind getPortKind() const { return PKind; }
  unsigned getLockWidth() const { return LockWidth; }

  uint64_t getGroup() const { return Group; }
  void setGroup(uint64_t G) { Group = G; }
  void setGroupForMemoryLock(unsigned AddressSpace, unsigned AliasGroup);

  void ConnectToSlave(BMLInterface *Slave);
  void ConnectToModuleInstance(VerilogModuleInstance *Inst, StringRef Prefix);
  BMLInterface *Clone(ModuleBuilder *Parent, StringRef Name,
                      PortKind portKind = BMLI_PK_None) const;
};

class LockSubsystem {
  ModuleBuilder *Parent;

  typedef SmallVector<BMLInterface*, 16> InterfaceList;
  typedef llvm::DenseMap<uint64_t, InterfaceList> GroupInterfaceListMap;
  GroupInterfaceListMap Interfaces;

  bool IsFinalized;

public:
  explicit LockSubsystem(ModuleBuilder *parent);

  void addInterface(BMLInterface *I);

  void Finalize();

private:
  void SynthesizeLock(InterfaceList &L);
};

} // namespace Synthesis

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CGLOCKSUBSYSTEM_H
