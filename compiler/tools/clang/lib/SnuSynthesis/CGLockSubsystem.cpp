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

#include "CGLockSubsystem.h"
#include "CGCommon.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

BMLInterface::BMLInterface(ModuleBuilder *parent, StringRef name,
                           unsigned lockWidth, BMLInterface::PortKind portKind)
  : Parent(parent), Name(name), PKind(portKind), LockWidth(lockWidth),
    Group((uint64_t)-1) {
  assert(LockWidth >= 1);
  VerilogSignalPortKind MtoS = SP_None;
  VerilogSignalPortKind StoM = SP_None;
  if (portKind == BMLI_PK_MasterPort) {
    MtoS = SP_Output;
    StoM = SP_Input;
  } else if (portKind == BMLI_PK_SlavePort) {
    MtoS = SP_Input;
    StoM = SP_Output;
  }
  LockID = Parent->CreateWire(MtoS, Name + "_lockid", LockWidth);
  LockRequestValid = Parent->CreateWire(MtoS, Name + "_lockrequestvalid");
  LockWaitRequest = Parent->CreateWire(StoM, Name + "_lockwaitrequest");
  LockResponseValid = Parent->CreateWire(StoM, Name + "_lockresponsevalid");
  LockWaitResponse = Parent->CreateWire(MtoS, Name + "_lockwaitresponse");
  UnlockID = Parent->CreateWire(MtoS, Name + "_unlockid", LockWidth);
  UnlockRequestValid = Parent->CreateWire(MtoS, Name + "_unlockrequestvalid");
  UnlockWaitRequest = Parent->CreateWire(StoM, Name + "_unlockwaitrequest");
  UnlockResponseValid = Parent->CreateWire(StoM, Name + "_unlockresponsevalid");
  UnlockWaitResponse = Parent->CreateWire(MtoS, Name + "_unlockwaitresponse");
}

void BMLInterface::setGroupForMemoryLock(unsigned AddressSpace,
                                         unsigned AliasGroup) {
  Group = ((uint64_t)AddressSpace << 32) | AliasGroup;
}

void BMLInterface::ConnectToSlave(BMLInterface *Slave) {
  assert(Parent == Slave->parent() &&
         "cannot connect interfaces in different modules");
  Parent->addAssignWire(Slave->lockid(), lockid());
  Parent->addAssignWire(Slave->lockrequestvalid(), lockrequestvalid());
  Parent->addAssignWire(lockwaitrequest(), Slave->lockwaitrequest());
  Parent->addAssignWire(lockresponsevalid(), Slave->lockresponsevalid());
  Parent->addAssignWire(Slave->lockwaitresponse(), lockwaitresponse());
  Parent->addAssignWire(Slave->unlockid(), unlockid());
  Parent->addAssignWire(Slave->unlockrequestvalid(), unlockrequestvalid());
  Parent->addAssignWire(unlockwaitrequest(), Slave->unlockwaitrequest());
  Parent->addAssignWire(unlockresponsevalid(), Slave->unlockresponsevalid());
  Parent->addAssignWire(Slave->unlockwaitresponse(), unlockwaitresponse());
}

void BMLInterface::ConnectToModuleInstance(VerilogModuleInstance *Inst,
                                           StringRef Prefix) {
  assert(Parent == Inst->getParent() &&
         "cannot connect an interface and a module instance in "
         "different modules");
  VerilogModule *Module = Inst->getTarget();
  std::string PrefixStr = (std::string)Prefix;
  assert(Module->hasArgument(PrefixStr + "_lockid"));
  assert(Module->hasArgument(PrefixStr + "_lockrequestvalid"));
  assert(Module->hasArgument(PrefixStr + "_lockwaitrequest"));
  assert(Module->hasArgument(PrefixStr + "_lockresponsevalid"));
  assert(Module->hasArgument(PrefixStr + "_lockwaitresponse"));
  assert(Module->hasArgument(PrefixStr + "_unlockid"));
  assert(Module->hasArgument(PrefixStr + "_unlockrequestvalid"));
  assert(Module->hasArgument(PrefixStr + "_unlockwaitrequest"));
  assert(Module->hasArgument(PrefixStr + "_unlockresponsevalid"));
  assert(Module->hasArgument(PrefixStr + "_unlockwaitresponse"));
  Inst->addArgument(PrefixStr + "_lockid", new VerilogSignalRef(lockid()));
  Inst->addArgument(PrefixStr + "_lockrequestvalid", new VerilogSignalRef(lockrequestvalid()));
  Inst->addArgument(PrefixStr + "_lockwaitrequest", new VerilogSignalRef(lockwaitrequest()));
  Inst->addArgument(PrefixStr + "_lockresponsevalid", new VerilogSignalRef(lockresponsevalid()));
  Inst->addArgument(PrefixStr + "_lockwaitresponse", new VerilogSignalRef(lockwaitresponse()));
  Inst->addArgument(PrefixStr + "_unlockid", new VerilogSignalRef(unlockid()));
  Inst->addArgument(PrefixStr + "_unlockrequestvalid", new VerilogSignalRef(unlockrequestvalid()));
  Inst->addArgument(PrefixStr + "_unlockwaitrequest", new VerilogSignalRef(unlockwaitrequest()));
  Inst->addArgument(PrefixStr + "_unlockresponsevalid", new VerilogSignalRef(unlockresponsevalid()));
  Inst->addArgument(PrefixStr + "_unlockwaitresponse", new VerilogSignalRef(unlockwaitresponse()));
}

BMLInterface *BMLInterface::Clone(ModuleBuilder *Parent, StringRef Name,
                                  PortKind portKind) const {
  BMLInterface *I = new BMLInterface(Parent, Name, getLockWidth(), portKind);
  I->setGroup(getGroup());
  return I;
}

LockSubsystem::LockSubsystem(ModuleBuilder *parent)
  : Parent(parent), IsFinalized(false) {
}

void LockSubsystem::addInterface(BMLInterface *I) {
  assert(!IsFinalized);
  assert(I->parent() == Parent);
  Interfaces[I->getGroup()].push_back(I);
}

void LockSubsystem::Finalize() {
  assert(!IsFinalized);
  IsFinalized = true;
  for (GroupInterfaceListMap::iterator I = Interfaces.begin(),
                                       E = Interfaces.end();
       I != E; ++I) {
    SynthesizeLock(I->second);
  }
}

void LockSubsystem::SynthesizeLock(InterfaceList &L) {
  assert(!L.empty());
  unsigned NumMasters = L.size();
  unsigned LockWidth = L[0]->getLockWidth();
  for (unsigned Index = 1; Index != NumMasters; ++Index) {
    assert(L[Index]->getLockWidth() == LockWidth);
  }

  VerilogSignal *LockIDArray = Parent->CreateTempWire("lock_array_lockid", LockWidth, NumMasters);
  VerilogSignal *LockRequestValidArray = Parent->CreateTempWire("lock_array_lockrequestvalid", 1, NumMasters);
  VerilogSignal *LockWaitRequestArray = Parent->CreateTempWire("lock_array_lockwaitrequest", 1, NumMasters);
  VerilogSignal *LockResponseValidArray = Parent->CreateTempWire("lock_array_lockresponsevalid", 1, NumMasters);
  VerilogSignal *LockWaitResponseArray = Parent->CreateTempWire("lock_array_lockwaitresponse", 1, NumMasters);
  VerilogSignal *UnlockIDArray = Parent->CreateTempWire("lock_array_unlockid", LockWidth, NumMasters);
  VerilogSignal *UnlockRequestValidArray = Parent->CreateTempWire("lock_array_unlockrequestvalid", 1, NumMasters);
  VerilogSignal *UnlockWaitRequestArray = Parent->CreateTempWire("lock_array_unlockwaitrequest", 1, NumMasters);
  VerilogSignal *UnlockResponseValidArray = Parent->CreateTempWire("lock_array_unlockresponsevalid", 1, NumMasters);
  VerilogSignal *UnlockWaitResponseArray = Parent->CreateTempWire("lock_array_unlockwaitresponse", 1, NumMasters);
  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    VerilogSignal *MasterLockID = L[Index]->lockid();
    VerilogSignal *MasterLockRequestValid = L[Index]->lockrequestvalid();
    VerilogSignal *MasterLockWaitRequest = L[Index]->lockwaitrequest();
    VerilogSignal *MasterLockResponseValid = L[Index]->lockresponsevalid();
    VerilogSignal *MasterLockWaitResponse = L[Index]->lockwaitresponse();
    VerilogSignal *MasterUnlockID = L[Index]->unlockid();
    VerilogSignal *MasterUnlockRequestValid = L[Index]->unlockrequestvalid();
    VerilogSignal *MasterUnlockWaitRequest = L[Index]->unlockwaitrequest();
    VerilogSignal *MasterUnlockResponseValid = L[Index]->unlockresponsevalid();
    VerilogSignal *MasterUnlockWaitResponse = L[Index]->unlockwaitresponse();

    VerilogSignalRef *SlaveLockID = CreateArrayElementOf(LockIDArray, Index);
    VerilogSignalRef *SlaveLockRequestValid = CreateArrayElementOf(LockRequestValidArray, Index);
    VerilogSignalRef *SlaveLockWaitRequest = CreateArrayElementOf(LockWaitRequestArray, Index);
    VerilogSignalRef *SlaveLockResponseValid = CreateArrayElementOf(LockResponseValidArray, Index);
    VerilogSignalRef *SlaveLockWaitResponse = CreateArrayElementOf(LockWaitResponseArray, Index);
    VerilogSignalRef *SlaveUnlockID = CreateArrayElementOf(UnlockIDArray, Index);
    VerilogSignalRef *SlaveUnlockRequestValid = CreateArrayElementOf(UnlockRequestValidArray, Index);
    VerilogSignalRef *SlaveUnlockWaitRequest = CreateArrayElementOf(UnlockWaitRequestArray, Index);
    VerilogSignalRef *SlaveUnlockResponseValid = CreateArrayElementOf(UnlockResponseValidArray, Index);
    VerilogSignalRef *SlaveUnlockWaitResponse = CreateArrayElementOf(UnlockWaitResponseArray, Index);

    Parent->addStmt(CreateAssignOf(SlaveLockID, MasterLockID));
    Parent->addStmt(CreateAssignOf(SlaveLockRequestValid, MasterLockRequestValid));
    Parent->addStmt(CreateAssignOf(MasterLockWaitRequest, SlaveLockWaitRequest));
    Parent->addStmt(CreateAssignOf(MasterLockResponseValid, SlaveLockResponseValid));
    Parent->addStmt(CreateAssignOf(SlaveLockWaitResponse, MasterLockWaitResponse));
    Parent->addStmt(CreateAssignOf(SlaveUnlockID, MasterUnlockID));
    Parent->addStmt(CreateAssignOf(SlaveUnlockRequestValid, MasterUnlockRequestValid));
    Parent->addStmt(CreateAssignOf(MasterUnlockWaitRequest, SlaveUnlockWaitRequest));
    Parent->addStmt(CreateAssignOf(MasterUnlockResponseValid, SlaveUnlockResponseValid));
    Parent->addStmt(CreateAssignOf(SlaveUnlockWaitResponse, MasterUnlockWaitResponse));
  }

  VerilogModuleInstance *Impl = Parent->CreateIPInstance("lock", "lock_inst");
  assert(Impl != NULL);
  Impl->addParamArgument("NUM_MASTERS", VerilogParamConst(NumMasters));
  Impl->addParamArgument("LOCK_WIDTH", VerilogParamConst(LockWidth));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef());
  Impl->addArgument("bys_lockid", new VerilogSignalRef(LockIDArray));
  Impl->addArgument("bys_lockrequestvalid", new VerilogSignalRef(LockRequestValidArray));
  Impl->addArgument("bys_lockwaitrequest", new VerilogSignalRef(LockWaitRequestArray));
  Impl->addArgument("bys_lockresponsevalid", new VerilogSignalRef(LockResponseValidArray));
  Impl->addArgument("bys_lockwaitresponse", new VerilogSignalRef(LockWaitResponseArray));
  Impl->addArgument("bys_unlockid", new VerilogSignalRef(UnlockIDArray));
  Impl->addArgument("bys_unlockrequestvalid", new VerilogSignalRef(UnlockRequestValidArray));
  Impl->addArgument("bys_unlockwaitrequest", new VerilogSignalRef(UnlockWaitRequestArray));
  Impl->addArgument("bys_unlockresponsevalid", new VerilogSignalRef(UnlockResponseValidArray));
  Impl->addArgument("bys_unlockwaitresponse", new VerilogSignalRef(UnlockWaitResponseArray));
}

} // namespace Synthesis

} // namespace snu

} // namespace clang
