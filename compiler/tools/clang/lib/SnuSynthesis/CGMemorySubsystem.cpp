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

#include "CGMemorySubsystem.h"
#include "CGCommon.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/MathExtras.h"
#include <algorithm>
#include <map>
#include <set>
#include <string>

namespace clang {

namespace snu {

namespace Synthesis {

// Avalon Memory-Mapped Interface

AMMInterfaceBase::AMMInterfaceBase(ModuleBuilder *parent, StringRef name,
                                   unsigned addrWidth, unsigned dataWidth,
                                   unsigned burstWidth,
                                   AMMInterfaceBase::PortKind portKind,
                                   bool Readable, bool Writable,
                                   bool HasWaitResponse)
  : Parent(parent), Name(name), PKind(portKind), MtoS(SP_None), StoM(SP_None),
    AddrWidth(addrWidth), DataWidth(dataWidth), BurstWidth(burstWidth),
    AddressSpace(0), AliasGroup(0), IsAtomic(false) {
  assert(Readable || Writable);
  assert(AddrWidth > 0 && DataWidth > 0);
  assert(DataWidth % 8 == 0);
  assert((DataWidth & (DataWidth - 1)) == 0); // DataWidth should be 2^k

  if (portKind == AMMI_PK_MasterPort) {
    MtoS = SP_Output;
    StoM = SP_Input;
  } else if (portKind == AMMI_PK_SlavePort) {
    MtoS = SP_Input;
    StoM = SP_Output;
  }

  Address = Parent->CreateWire(MtoS, Name + "_address", AddrWidth);
  if (Readable) {
    Read = Parent->CreateWire(MtoS, Name + "_read");
    ReadData = Parent->CreateWire(StoM, Name + "_readdata", DataWidth);
    ReadDataValid = Parent->CreateWire(StoM, Name + "_readdatavalid");
  } else {
    Read = NULL;
    ReadData = NULL;
    ReadDataValid = NULL;
  }
  if (Writable) {
    Write = Parent->CreateWire(MtoS, Name + "_write");
    WriteData = Parent->CreateWire(MtoS, Name + "_writedata", DataWidth);
    ByteEnable = Parent->CreateWire(MtoS, Name + "_byteenable", DataWidth / 8);
  } else {
    Write = NULL;
    WriteData = NULL;
    ByteEnable = NULL;
  }
  if (BurstWidth > 0) {
    BurstCount = Parent->CreateWire(MtoS, Name + "_burstcount", BurstWidth);
  } else {
    BurstCount = NULL;
  }
  WaitRequest = Parent->CreateWire(StoM, Name + "_waitrequest");
  if (HasWaitResponse) {
    WaitResponse = Parent->CreateWire(MtoS, Name + "_waitresponse");
  } else {
    WaitResponse = NULL;
  }
}

void AMMInterfaceBase::ConnectToSlave(AMMInterfaceBase *Slave) {
  assert(parent() == Slave->parent() &&
         "cannot connect interfaces in different modules");
  assert(Slave->getDataWidth() >= getDataWidth());
  if (Slave->getDataWidth() > getDataWidth()) {
    AMMInterfaceBase M(*this);
    AMMInterfaceBase S(*Slave);
    VerilogModuleInstance *Impl;
    if (isReadable() && isWritable()) {
      Impl = parent()->CreateIPInstance("dc_data_width_adapter", "dc_dwadapter");
    } else if (isReadable()) {
      Impl = parent()->CreateIPInstance("dc_data_width_adapter_ro", "dc_dwadapter");
    } else if (isWritable()) {
      Impl = parent()->CreateIPInstance("dc_data_width_adapter_wo", "dc_dwadapter");
    } else {
      llvm_unreachable("impossible case");
    }
    assert(Impl != NULL);
    Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(getAddressWidth()));
    Impl->addParamArgument("MASTER_DATA_WIDTH", VerilogParamConst(getDataWidth()));
    Impl->addParamArgument("SLAVE_DATA_WIDTH", VerilogParamConst(Slave->getDataWidth()));
    Impl->addArgument("clk", parent()->CreateClockRef());
    Impl->addArgument("rstn", parent()->CreateRstnRef());
    M.ConnectToModuleInstanceAsMaster(Impl, "avs");
    S.ConnectToModuleInstanceAsSlave(Impl, "avm");

  } else {
    parent()->addAssignWire(Slave->address(), address());
    if (isReadable()) {
      assert(Slave->isReadable());
      parent()->addAssignWire(Slave->read(), read());
      parent()->addAssignWire(readdata(), Slave->readdata());
      parent()->addAssignWire(readdatavalid(), Slave->readdatavalid());
    } else if (Slave->isReadable()) {
      parent()->addAssignWire(Slave->read(), CreateZero());
    }
    if (isWritable()) {
      assert(Slave->isWritable());
      parent()->addAssignWire(Slave->write(), write());
      parent()->addAssignWire(Slave->writedata(), writedata());
      parent()->addAssignWire(Slave->byteenable(), byteenable());
    } else if (Slave->isWritable()) {
      parent()->addAssignWire(Slave->write(), CreateZero());
      parent()->addAssignWire(Slave->writedata(), CreateDecimalZero());
      parent()->addAssignWire(Slave->byteenable(), CreateDecimalZero());
    }
    if (isBurstAccessable()) {
      assert(Slave->isBurstAccessable());
      parent()->addAssignWire(Slave->burstcount(), burstcount());
    } else if (Slave->isBurstAccessable()) {
      parent()->addAssignWire(Slave->burstcount(), CreateDecimalOne());
    }
    parent()->addAssignWire(waitrequest(), Slave->waitrequest());
    if (isHandshakable()) {
      assert(Slave->isHandshakable());
      parent()->addAssignWire(Slave->waitresponse(), waitresponse());
    } else if (Slave->isHandshakable()) {
      parent()->addAssignWire(Slave->waitresponse(), CreateZero());
    }
  }
}

void AMMInterfaceBase::ConnectToModuleInstanceAsMaster(
    VerilogModuleInstance *Slave, StringRef Prefix) {
  assert(parent() == Slave->getParent() &&
         "cannot connect an interface and a module instance in "
         "different modules");

  VerilogModule *SlaveModule = Slave->getTarget();
  std::string PrefixStr = (std::string)Prefix;

  assert(SlaveModule->hasArgument(PrefixStr + "_address"));
  Slave->addArgument(PrefixStr + "_address", new VerilogSignalRef(address()));
  if (isReadable()) {
    assert(SlaveModule->hasArgument(PrefixStr + "_read"));
    assert(SlaveModule->hasArgument(PrefixStr + "_readdata"));
    assert(SlaveModule->hasArgument(PrefixStr + "_readdatavalid"));
    Slave->addArgument(PrefixStr + "_read", new VerilogSignalRef(read()));
    Slave->addArgument(PrefixStr + "_readdata", new VerilogSignalRef(readdata()));
    Slave->addArgument(PrefixStr + "_readdatavalid", new VerilogSignalRef(readdatavalid()));
  } else if (SlaveModule->hasArgument(PrefixStr + "_read")) {
    Slave->addArgument(PrefixStr + "_read", CreateZero());
  }
  if (isWritable()) {
    assert(SlaveModule->hasArgument(PrefixStr + "_write"));
    assert(SlaveModule->hasArgument(PrefixStr + "_writedata"));
    assert(SlaveModule->hasArgument(PrefixStr + "_byteenable"));
    Slave->addArgument(PrefixStr + "_write", new VerilogSignalRef(write()));
    Slave->addArgument(PrefixStr + "_writedata", new VerilogSignalRef(writedata()));
    Slave->addArgument(PrefixStr + "_byteenable", new VerilogSignalRef(byteenable()));
  } else if (SlaveModule->hasArgument(PrefixStr + "_write")) {
    assert(SlaveModule->hasArgument(PrefixStr + "_writedata"));
    assert(SlaveModule->hasArgument(PrefixStr + "_byteenable"));
    Slave->addArgument(PrefixStr + "_write", CreateZero());
    Slave->addArgument(PrefixStr + "_writedata", CreateDecimalZero());
    Slave->addArgument(PrefixStr + "_byteenable", CreateDecimalZero());
  }
  if (isBurstAccessable()) {
    assert(SlaveModule->hasArgument(PrefixStr + "_burstcount"));
    Slave->addArgument(PrefixStr + "_burstcount", new VerilogSignalRef(burstcount()));
  } else if (SlaveModule->hasArgument(PrefixStr + "_burstcount")) {
    Slave->addArgument(PrefixStr + "_burstcount", CreateDecimalOne());
  }
  assert(SlaveModule->hasArgument(PrefixStr + "_waitrequest"));
  Slave->addArgument(PrefixStr + "_waitrequest", new VerilogSignalRef(waitrequest()));
  if (isHandshakable()) {
    assert(SlaveModule->hasArgument(PrefixStr + "_waitresponse"));
    Slave->addArgument(PrefixStr + "_waitresponse", new VerilogSignalRef(waitresponse()));
  } else if (SlaveModule->hasArgument(PrefixStr + "_waitresponse")) {
    Slave->addArgument(PrefixStr + "_waitresponse", CreateZero());
  }
}

void AMMInterfaceBase::ConnectToModuleInstanceAsSlave(
    VerilogModuleInstance *Master, StringRef Prefix) {
  assert(parent() == Master->getParent() &&
         "cannot connect an interface and a module instance in "
         "different modules");

  VerilogModule *MasterModule = Master->getTarget();
  std::string PrefixStr = (std::string)Prefix;

  assert(MasterModule->hasArgument(PrefixStr + "_address"));
  Master->addArgument(PrefixStr + "_address", new VerilogSignalRef(address()));
  if (MasterModule->hasArgument(PrefixStr + "_read")) {
    assert(isReadable());
    assert(MasterModule->hasArgument(PrefixStr + "_readdata"));
    assert(MasterModule->hasArgument(PrefixStr + "_readdatavalid"));
    Master->addArgument(PrefixStr + "_read", new VerilogSignalRef(read()));
    Master->addArgument(PrefixStr + "_readdata", new VerilogSignalRef(readdata()));
    Master->addArgument(PrefixStr + "_readdatavalid", new VerilogSignalRef(readdatavalid()));
  } else if (isReadable()) {
    parent()->addAssignWire(read(), CreateZero());
  }
  if (MasterModule->hasArgument(PrefixStr + "_write")) {
    assert(isWritable());
    assert(MasterModule->hasArgument(PrefixStr + "_writedata"));
    assert(MasterModule->hasArgument(PrefixStr + "_byteenable"));
    Master->addArgument(PrefixStr + "_write", new VerilogSignalRef(write()));
    Master->addArgument(PrefixStr + "_writedata", new VerilogSignalRef(writedata()));
    Master->addArgument(PrefixStr + "_byteenable", new VerilogSignalRef(byteenable()));
  } else if (isWritable()) {
    parent()->addAssignWire(write(), CreateZero());
    parent()->addAssignWire(writedata(), CreateDecimalZero());
    parent()->addAssignWire(byteenable(), CreateDecimalZero());
  }
  if (MasterModule->hasArgument(PrefixStr + "_burstcount")) {
    assert(isBurstAccessable());
    assert(MasterModule->hasArgument(PrefixStr + "_burstcount"));
    Master->addArgument(PrefixStr + "_burstcount", new VerilogSignalRef(burstcount()));
  } else if (isBurstAccessable()) {
    parent()->addAssignWire(burstcount(), CreateDecimalOne());
  }
  assert(MasterModule->hasArgument(PrefixStr + "_waitrequest"));
  Master->addArgument(PrefixStr + "_waitrequest", new VerilogSignalRef(waitrequest()));
  if (MasterModule->hasArgument(PrefixStr + "_waitresponse")) {
    assert(isHandshakable());
    Master->addArgument(PrefixStr + "_waitresponse", new VerilogSignalRef(waitresponse()));
  } else if (isHandshakable()) {
    parent()->addAssignWire(waitresponse(), CreateZero());
  }
}

void AMMInterfaceBase::ConnectToAXISlave(AXIInterface *Slave) {
  assert(parent() == Slave->parent() &&
         "cannot connect interfaces in different modules");
  assert(getAddressWidth() == Slave->getAddressWidth());
  assert(getDataWidth() == Slave->getDataWidth());
  VerilogModuleInstance *Impl = parent()->CreateIPInstance(
      "amm_to_axi_converter_nonburst", "mem_converter_inst");
  assert(Impl != NULL);
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(getAddressWidth()));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(getDataWidth()));
  Impl->addParamArgument("BURST_WIDTH", VerilogParamConst(getBurstWidth()));
  Impl->addArgument("clk", parent()->CreateClockRef());
  Impl->addArgument("rstn", parent()->CreateRstnRef());
  ConnectToModuleInstanceAsMaster(Impl, "amm");
  Slave->ConnectToModuleInstance(Impl, "axi");
}

AMMInterfaceBase *AMMInterfaceBase::Clone(ModuleBuilder *Parent, StringRef Name,
                                          PortKind portKind) const {
  AMMInterfaceBase *New = new AMMInterfaceBase(Parent, Name, getAddressWidth(),
                                               getDataWidth(), getBurstWidth(),
                                               portKind, isReadable(),
                                               isWritable(), isHandshakable());
  New->AnnotateAddressSpace(getAddressSpace());
  New->AnnotateAliasGroup(getAliasGroup());
  if (isAtomic()) {
    New->AnnotateAsAtomic();
  }
  return New;
}

AMMInterfaceBase *AMMInterfaceBase::ResizedClone(ModuleBuilder *Parent,
                                                 StringRef Name,
                                                 unsigned NewDataWidth,
                                                 PortKind portKind) const {
  AMMInterfaceBase *New = new AMMInterfaceBase(Parent, Name, getAddressWidth(),
                                               NewDataWidth, getBurstWidth(),
                                               portKind, isReadable(),
                                               isWritable(), isHandshakable());
  New->AnnotateAddressSpace(getAddressSpace());
  New->AnnotateAliasGroup(getAliasGroup());
  if (isAtomic()) {
    New->AnnotateAsAtomic();
  }
  return New;
}

// AXI Interface

AXIInterface::AXIInterface(ModuleBuilder *parent, StringRef name,
                           unsigned addrWidth, unsigned dataWidth,
                           AXIInterface::PortKind portKind)
  : Parent(parent), Name(name), PKind(portKind), AddrWidth(addrWidth),
    DataWidth(dataWidth), AddressSpace(0), AliasGroup(0) {
  assert(DataWidth % 8 == 0);

  VerilogSignalPortKind MtoS = SP_None;
  VerilogSignalPortKind StoM = SP_None;
  if (portKind == AXII_PK_MasterPort) {
    MtoS = SP_Output;
    StoM = SP_Input;
  } else if (portKind == AXII_PK_SlavePort) {
    MtoS = SP_Input;
    StoM = SP_Output;
  }

  ARAddr = Parent->CreateWire(MtoS, Name + "_araddr", AddrWidth);
  ARBurst = Parent->CreateWire(MtoS, Name + "_arburst", 2);
  ARCache = Parent->CreateWire(MtoS, Name + "_arcache", 4);
  ARLen = Parent->CreateWire(MtoS, Name + "_arlen", 8);
  ARReady = Parent->CreateWire(StoM, Name + "_arready");
  ARSize = Parent->CreateWire(MtoS, Name + "_arsize", 3);
  ARValid = Parent->CreateWire(MtoS, Name + "_arvalid");

  AWAddr = Parent->CreateWire(MtoS, Name + "_awaddr", AddrWidth);
  AWBurst = Parent->CreateWire(MtoS, Name + "_awburst", 2);
  AWCache = Parent->CreateWire(MtoS, Name + "_awcache", 4);
  AWLen = Parent->CreateWire(MtoS, Name + "_awlen", 8);
  AWReady = Parent->CreateWire(StoM, Name + "_awready");
  AWSize = Parent->CreateWire(MtoS, Name + "_awsize", 3);
  AWValid = Parent->CreateWire(MtoS, Name + "_awvalid");

  BReady = Parent->CreateWire(MtoS, Name + "_bready");
  BResp = Parent->CreateWire(StoM, Name + "_bresp", 2);
  BValid = Parent->CreateWire(StoM, Name + "_bvalid");

  RData = Parent->CreateWire(StoM, Name + "_rdata", DataWidth);
  RLast = Parent->CreateWire(StoM, Name + "_rlast");
  RReady = Parent->CreateWire(MtoS, Name + "_rready");
  RResp = Parent->CreateWire(StoM, Name + "_rresp", 2);
  RValid = Parent->CreateWire(StoM, Name + "_rvalid");

  WData = Parent->CreateWire(MtoS, Name + "_wdata", DataWidth);
  WLast = Parent->CreateWire(MtoS, Name + "_wlast");
  WReady = Parent->CreateWire(StoM, Name + "_wready");
  WStrb = Parent->CreateWire(MtoS, Name + "_wstrb", DataWidth / 8);
  WValid = Parent->CreateWire(MtoS, Name + "_wvalid");
}

void AXIInterface::ConnectToSlave(AXIInterface *Slave) {
  assert(parent() == Slave->parent() &&
         "cannot connect interfaces in different modules");
  assert(getDataWidth() == Slave->getDataWidth());
  parent()->addAssignWire(Slave->araddr(), araddr());
  parent()->addAssignWire(Slave->arburst(), arburst());
  parent()->addAssignWire(Slave->arcache(), arcache());
  parent()->addAssignWire(Slave->arlen(), arlen());
  parent()->addAssignWire(arready(), Slave->arready());
  parent()->addAssignWire(Slave->arsize(), arsize());
  parent()->addAssignWire(Slave->arvalid(), arvalid());
  parent()->addAssignWire(Slave->awaddr(), awaddr());
  parent()->addAssignWire(Slave->awburst(), awburst());
  parent()->addAssignWire(Slave->awcache(), awcache());
  parent()->addAssignWire(Slave->awlen(), awlen());
  parent()->addAssignWire(awready(), Slave->awready());
  parent()->addAssignWire(Slave->awsize(), awsize());
  parent()->addAssignWire(Slave->awvalid(), awvalid());
  parent()->addAssignWire(Slave->bready(), bready());
  parent()->addAssignWire(bresp(), Slave->bresp());
  parent()->addAssignWire(bvalid(), Slave->bvalid());
  parent()->addAssignWire(rdata(), Slave->rdata());
  parent()->addAssignWire(rlast(), Slave->rlast());
  parent()->addAssignWire(Slave->rready(), rready());
  parent()->addAssignWire(rresp(), Slave->rresp());
  parent()->addAssignWire(rvalid(), Slave->rvalid());
  parent()->addAssignWire(Slave->wdata(), wdata());
  parent()->addAssignWire(Slave->wlast(), wlast());
  parent()->addAssignWire(wready(), Slave->wready());
  parent()->addAssignWire(Slave->wstrb(), wstrb());
  parent()->addAssignWire(Slave->wvalid(), wvalid());
}

void AXIInterface::ConnectToModuleInstance(VerilogModuleInstance *Inst,
                                           StringRef Prefix) {
  assert(parent() == Inst->getParent() &&
         "cannot connect an interface and a module instance in "
         "different modules");

  VerilogModule *Module = Inst->getTarget();
  std::string PrefixStr = (std::string)Prefix;

  assert(Module->hasArgument(PrefixStr + "_araddr"));
  assert(Module->hasArgument(PrefixStr + "_arburst"));
  assert(Module->hasArgument(PrefixStr + "_arcache"));
  assert(Module->hasArgument(PrefixStr + "_arlen"));
  assert(Module->hasArgument(PrefixStr + "_arready"));
  assert(Module->hasArgument(PrefixStr + "_arsize"));
  assert(Module->hasArgument(PrefixStr + "_arvalid"));
  assert(Module->hasArgument(PrefixStr + "_awaddr"));
  assert(Module->hasArgument(PrefixStr + "_awburst"));
  assert(Module->hasArgument(PrefixStr + "_awcache"));
  assert(Module->hasArgument(PrefixStr + "_awlen"));
  assert(Module->hasArgument(PrefixStr + "_awready"));
  assert(Module->hasArgument(PrefixStr + "_awsize"));
  assert(Module->hasArgument(PrefixStr + "_awvalid"));
  assert(Module->hasArgument(PrefixStr + "_bready"));
  assert(Module->hasArgument(PrefixStr + "_bresp"));
  assert(Module->hasArgument(PrefixStr + "_bvalid"));
  assert(Module->hasArgument(PrefixStr + "_rdata"));
  assert(Module->hasArgument(PrefixStr + "_rlast"));
  assert(Module->hasArgument(PrefixStr + "_rready"));
  assert(Module->hasArgument(PrefixStr + "_rresp"));
  assert(Module->hasArgument(PrefixStr + "_rvalid"));
  assert(Module->hasArgument(PrefixStr + "_wdata"));
  assert(Module->hasArgument(PrefixStr + "_wlast"));
  assert(Module->hasArgument(PrefixStr + "_wready"));
  assert(Module->hasArgument(PrefixStr + "_wstrb"));
  assert(Module->hasArgument(PrefixStr + "_wvalid"));
  Inst->addArgument(PrefixStr + "_araddr", new VerilogSignalRef(araddr()));
  Inst->addArgument(PrefixStr + "_arburst", new VerilogSignalRef(arburst()));
  Inst->addArgument(PrefixStr + "_arcache", new VerilogSignalRef(arcache()));
  Inst->addArgument(PrefixStr + "_arlen", new VerilogSignalRef(arlen()));
  Inst->addArgument(PrefixStr + "_arready", new VerilogSignalRef(arready()));
  Inst->addArgument(PrefixStr + "_arsize", new VerilogSignalRef(arsize()));
  Inst->addArgument(PrefixStr + "_arvalid", new VerilogSignalRef(arvalid()));
  Inst->addArgument(PrefixStr + "_awaddr", new VerilogSignalRef(awaddr()));
  Inst->addArgument(PrefixStr + "_awburst", new VerilogSignalRef(awburst()));
  Inst->addArgument(PrefixStr + "_awcache", new VerilogSignalRef(awcache()));
  Inst->addArgument(PrefixStr + "_awlen", new VerilogSignalRef(awlen()));
  Inst->addArgument(PrefixStr + "_awready", new VerilogSignalRef(awready()));
  Inst->addArgument(PrefixStr + "_awsize", new VerilogSignalRef(awsize()));
  Inst->addArgument(PrefixStr + "_awvalid", new VerilogSignalRef(awvalid()));
  Inst->addArgument(PrefixStr + "_bready", new VerilogSignalRef(bready()));
  Inst->addArgument(PrefixStr + "_bresp", new VerilogSignalRef(bresp()));
  Inst->addArgument(PrefixStr + "_bvalid", new VerilogSignalRef(bvalid()));
  Inst->addArgument(PrefixStr + "_rdata", new VerilogSignalRef(rdata()));
  Inst->addArgument(PrefixStr + "_rlast", new VerilogSignalRef(rlast()));
  Inst->addArgument(PrefixStr + "_rready", new VerilogSignalRef(rready()));
  Inst->addArgument(PrefixStr + "_rresp", new VerilogSignalRef(rresp()));
  Inst->addArgument(PrefixStr + "_rvalid", new VerilogSignalRef(rvalid()));
  Inst->addArgument(PrefixStr + "_wdata", new VerilogSignalRef(wdata()));
  Inst->addArgument(PrefixStr + "_wlast", new VerilogSignalRef(wlast()));
  Inst->addArgument(PrefixStr + "_wready", new VerilogSignalRef(wready()));
  Inst->addArgument(PrefixStr + "_wstrb", new VerilogSignalRef(wstrb()));
  Inst->addArgument(PrefixStr + "_wvalid", new VerilogSignalRef(wvalid()));
}

AXIInterface *AXIInterface::Clone(ModuleBuilder *Parent, StringRef Name,
                                  PortKind portKind) const {
  AXIInterface *I = new AXIInterface(Parent, Name, getAddressWidth(),
                                     getDataWidth(), portKind);
  I->AnnotateAddressSpace(getAddressSpace());
  I->AnnotateAliasGroup(getAliasGroup());
  return I;
}

// Global Memory Subsystem

GlobalMemorySubsystem::GlobalMemorySubsystem(ModuleBuilder *parent)
  : Parent(parent), NumConsumedPorts(0), IsFinalized(false) {
}

void GlobalMemorySubsystem::addInterface(unsigned Datapath,
                                         AMMInterfaceBase *I) {
  assert(!IsFinalized);
  assert(I->parent() == Parent);
  assert(I->getAddressSpace() == LangAS::opencl_global);
  InterfaceGroupID Key;
  Key.Parts.PerBufferIndex.Datapath = Datapath;
  Key.Parts.PerBufferIndex.AliasGroup = I->getAliasGroup();
  Interfaces[Key.Raw].push_back(I);
  if (I->isAtomic()) {
    AtomicGroups.insert(Key.Raw);
  }
}

void GlobalMemorySubsystem::addGlobalMemoryPort(AMMInterface *M) {
  assert(!IsFinalized);
  assert(M->parent() == Parent);
  AMMPorts.push_back(M);
}

void GlobalMemorySubsystem::addGlobalMemoryPort(AXIInterface *M) {
  assert(!IsFinalized);
  assert(M->parent() == Parent);
  AXIPorts.push_back(M);
}

void GlobalMemorySubsystem::Finalize(VerilogSignal *Clean,
                                     VerilogSignal *Cleaned) {
  assert(!IsFinalized);
  IsFinalized = true;

  for (std::set<uint64_t>::iterator I = AtomicGroups.begin(),
                                    E = AtomicGroups.end();
       I != E; ++I) {
    InterfaceGroupID CurGroup = *I;
    InterfaceGroupID NewGroup = *I;
    NewGroup.Parts.PerBufferIndex.Datapath = 0;
    if (CurGroup != NewGroup) {
      Interfaces[NewGroup.Raw].append(Interfaces[CurGroup.Raw].begin(),
                                      Interfaces[CurGroup.Raw].end());
      Interfaces.erase(CurGroup.Raw);
    }
  }

  for (GroupInterfaceListMap::iterator I = Interfaces.begin(),
                                       E = Interfaces.end();
       I != E; ++I) {
    SynthesizeArbiter(I->second);
  }

  unsigned NumRequiredPorts = ComputeNumRequiredPorts();
  if (NumRequiredPorts > AMMPorts.size()) {
    if (!AMMPorts.empty()) {
      SynthesizeMoreAMMPorts(NumRequiredPorts);
    } else if (!AXIPorts.empty()) {
      SynthesizeMoreAXIPorts(NumRequiredPorts);
      ConvertAXItoAMMPorts();
    } else {
      llvm_unreachable("impossible");
    }
  }
  assert(AMMPorts.size() >= NumRequiredPorts);

  for (GroupInterfaceListMap::iterator I = Interfaces.begin(),
                                       E = Interfaces.end();
       I != E; ++I) {
    SynthesizeCaches(I->second, Clean);
  }
  SynthesizeCleaned(Cleaned);

  while (NumConsumedPorts < AMMPorts.size()) {
    AMMInterface *Port = AMMPorts[NumConsumedPorts++];
    Parent->addAssignWire(Port->address(), CreateZero(Port->getAddressWidth()));
    if (Port->isBurstAccessable()) {
      Parent->addAssignWire(Port->burstcount(), CreateZero(Port->getBurstWidth()));
    }
    Parent->addAssignWire(Port->read(), CreateZero());
    Parent->addAssignWire(Port->write(), CreateZero());
    Parent->addAssignWire(Port->writedata(), CreateZero(Port->getDataWidth()));
    Parent->addAssignWire(Port->byteenable(), CreateZero(Port->getDataWidth() / 8));
  }
}

void GlobalMemorySubsystem::SynthesizeArbiter(InterfaceList &L, unsigned NumSlaves) {
  assert(!L.empty());
  unsigned NumMasters = L.size();
  unsigned AddressWidth = L[0]->getAddressWidth();
  unsigned DataWidth = L[0]->getDataWidth();
  unsigned BurstWidth = L[0]->getBurstWidth();
  bool Read = L[0]->isReadable();
  bool Write = L[0]->isWritable();
  bool Handshakable = L[0]->isHandshakable();
  unsigned AddressSpace = L[0]->getAddressSpace();
  unsigned AliasGroup = L[0]->getAliasGroup();
  bool IsAtomic = L[0]->isAtomic();
  for (unsigned Index = 1; Index != NumMasters; ++Index) {
    assert(L[Index]->parent() == Parent);
    assert(L[Index]->getAddressWidth() == AddressWidth);
    DataWidth = std::max(DataWidth, L[Index]->getDataWidth());
    assert(L[Index]->getBurstWidth() == BurstWidth);
    Read |= L[Index]->isReadable();
    Write |= L[Index]->isWritable();
    Handshakable |= L[Index]->isHandshakable();
    assert(L[Index]->getAddressSpace() == AddressSpace);
    assert(L[Index]->getAliasGroup() == AliasGroup);
    IsAtomic |= L[Index]->isAtomic();
  }
  assert(!Read || Handshakable); // Read -> Handshakable

  if (NumSlaves == 0) {
    // TODO: demultiplexing
    NumSlaves = 1;
  }

  if (NumMasters == 1 && NumSlaves == 1) {
    return;
  }

  InterfaceList Masters = L;
  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    if (Masters[Index]->getDataWidth() != DataWidth) {
      AMMInterfaceBase *NewMaster = Masters[Index]->ResizedClone(
          Parent, Parent->AcquireNameWithPrefix("mem_resized"), DataWidth);
      Masters[Index]->ConnectToSlave(NewMaster);
      Masters[Index] = NewMaster;
    }
  }

  InterfaceList Slaves;
  AMMInterfaceBase *FirstSlave;
  if (Read && Write) {
    FirstSlave = new HAMMInterface(
        Parent, Parent->AcquireNameWithPrefix("mem_arbitered"), AddressWidth,
        DataWidth, BurstWidth);
  } else if (Read) {
    FirstSlave = new HAMMReadOnlyInterface(
        Parent, Parent->AcquireNameWithPrefix("mem_arbitered"), AddressWidth,
        DataWidth, BurstWidth);
  } else {
    assert(Write);
    FirstSlave = new AMMWriteOnlyInterface(
        Parent, Parent->AcquireNameWithPrefix("mem_arbitered"), AddressWidth,
        DataWidth, BurstWidth);
  }
  FirstSlave->AnnotateAddressSpace(AddressSpace);
  FirstSlave->AnnotateAliasGroup(AliasGroup);
  if (IsAtomic) {
    FirstSlave->AnnotateAsAtomic();
  }
  Slaves.push_back(FirstSlave);
  for (unsigned Index = 1; Index != NumSlaves; ++Index) {
    Slaves.push_back(FirstSlave->Clone(
        Parent, Parent->AcquireNameWithPrefix("mem_arbitered")));
  }

  VerilogSignal *MasterAddress = NULL;
  VerilogSignal *MasterRead = NULL;
  VerilogSignal *MasterReadData = NULL;
  VerilogSignal *MasterReadDataValid = NULL;
  VerilogSignal *MasterWrite = NULL;
  VerilogSignal *MasterWriteData = NULL;
  VerilogSignal *MasterByteEnable = NULL;
  VerilogSignal *MasterWaitRequest = NULL;
  VerilogSignal *MasterWaitResponse = NULL;

  MasterAddress = Parent->CreateTempWire("mem_array_address", AddressWidth, NumMasters);
  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    VerilogSignalRef *MasterAddressRef = CreateArrayElementOf(MasterAddress, Index);
    Parent->addStmt(CreateAssignOf(MasterAddressRef, Masters[Index]->address()));
  }
  if (Read) {
    MasterRead = Parent->CreateTempWire("mem_array_read", 1, NumMasters);
    MasterReadData = Parent->CreateTempWire("mem_array_readdata", DataWidth, NumMasters);
    MasterReadDataValid = Parent->CreateTempWire("mem_array_readdatavalid", 1, NumMasters);
    for (unsigned Index = 0; Index != NumMasters; ++Index) {
      if (Masters[Index]->isReadable()) {
        VerilogSignalRef *MasterReadRef = CreateArrayElementOf(MasterRead, Index);
        VerilogSignalRef *MasterReadDataRef = CreateArrayElementOf(MasterReadData, Index);
        VerilogSignalRef *MasterReadDataValidRef = CreateArrayElementOf(MasterReadDataValid, Index);
        Parent->addStmt(CreateAssignOf(MasterReadRef, Masters[Index]->read()));
        Parent->addStmt(CreateAssignOf(Masters[Index]->readdata(), MasterReadDataRef));
        Parent->addStmt(CreateAssignOf(Masters[Index]->readdatavalid(), MasterReadDataValidRef));
      } else {
        VerilogSignalRef *MasterReadRef = CreateArrayElementOf(MasterRead, Index);
        Parent->addStmt(CreateAssignOf(MasterReadRef, CreateZero()));
      }
    }
  }
  if (Write) {
    MasterWrite = Parent->CreateTempWire("mem_array_write", 1, NumMasters);
    MasterWriteData = Parent->CreateTempWire("mem_array_writedata", DataWidth, NumMasters);
    MasterByteEnable = Parent->CreateTempWire("mem_array_byteenable", DataWidth / 8, NumMasters);
    for (unsigned Index = 0; Index != NumMasters; ++Index) {
      VerilogSignalRef *MasterWriteRef = CreateArrayElementOf(MasterWrite, Index);
      VerilogSignalRef *MasterWriteDataRef = CreateArrayElementOf(MasterWriteData, Index);
      VerilogSignalRef *MasterByteEnableRef = CreateArrayElementOf(MasterByteEnable, Index);
      if (Masters[Index]->isWritable()) {
        Parent->addStmt(CreateAssignOf(MasterWriteRef, Masters[Index]->write()));
        Parent->addStmt(CreateAssignOf(MasterWriteDataRef, Masters[Index]->writedata()));
        Parent->addStmt(CreateAssignOf(MasterByteEnableRef, Masters[Index]->byteenable()));
      } else {
        Parent->addStmt(CreateAssignOf(MasterWriteRef, CreateZero()));
        Parent->addStmt(CreateAssignOf(MasterWriteDataRef, CreateZero(DataWidth)));
        Parent->addStmt(CreateAssignOf(MasterByteEnableRef, CreateZero(DataWidth / 8)));
      }
    }
  }
  MasterWaitRequest = Parent->CreateTempWire("mem_array_waitrequest", 1, NumMasters);
  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    VerilogSignalRef *MasterWaitRequestRef = CreateArrayElementOf(MasterWaitRequest, Index);
    Parent->addStmt(CreateAssignOf(Masters[Index]->waitrequest(), MasterWaitRequestRef));
  }
  if (Handshakable) {
    MasterWaitResponse = Parent->CreateTempWire("mem_array_waitresponse", 1, NumMasters);
    for (unsigned Index = 0; Index != NumMasters; ++Index) {
      VerilogSignalRef *MasterWaitResponseRef = CreateArrayElementOf(MasterWaitResponse, Index);
      if (Masters[Index]->isHandshakable()) {
        Parent->addStmt(CreateAssignOf(MasterWaitResponseRef, Masters[Index]->waitresponse()));
      } else {
        Parent->addStmt(CreateAssignOf(MasterWaitResponseRef, CreateZero()));
      }
    }
  }

  VerilogSignal *SlaveAddress = NULL;
  VerilogSignal *SlaveRead = NULL;
  VerilogSignal *SlaveReadData = NULL;
  VerilogSignal *SlaveReadDataValid = NULL;
  VerilogSignal *SlaveWrite = NULL;
  VerilogSignal *SlaveWriteData = NULL;
  VerilogSignal *SlaveByteEnable = NULL;
  VerilogSignal *SlaveWaitRequest = NULL;
  VerilogSignal *SlaveWaitResponse = NULL;

  if (NumSlaves == 1) {
    AMMInterfaceBase *FirstSlave = Slaves.front();
    SlaveAddress = FirstSlave->address();
    SlaveRead = FirstSlave->read();
    SlaveReadData = FirstSlave->readdata();
    SlaveReadDataValid = FirstSlave->readdatavalid();
    SlaveWrite = FirstSlave->write();
    SlaveWriteData = FirstSlave->writedata();
    SlaveByteEnable = FirstSlave->byteenable();
    SlaveWaitRequest = FirstSlave->waitrequest();
    SlaveWaitResponse = FirstSlave->waitresponse();

  } else {
    SlaveAddress = Parent->CreateTempWire("mem_array_address", AddressWidth, NumSlaves);
    for (unsigned Index = 0; Index != NumSlaves; ++Index) {
      VerilogSignalRef *SlaveAddressRef = CreateArrayElementOf(SlaveAddress, Index);
      Parent->addStmt(CreateAssignOf(Slaves[Index]->address(), SlaveAddressRef));
    }
    if (Read) {
      SlaveRead = Parent->CreateTempWire("mem_array_read", 1, NumSlaves);
      SlaveReadData = Parent->CreateTempWire("mem_array_readdata", DataWidth, NumSlaves);
      SlaveReadDataValid = Parent->CreateTempWire("mem_array_readdatavalid", 1, NumSlaves);
      for (unsigned Index = 0; Index != NumSlaves; ++Index) {
        VerilogSignalRef *SlaveReadRef = CreateArrayElementOf(SlaveRead, Index);
        VerilogSignalRef *SlaveReadDataRef = CreateArrayElementOf(SlaveReadData, Index);
        VerilogSignalRef *SlaveReadDataValidRef = CreateArrayElementOf(SlaveReadDataValid, Index);
        Parent->addStmt(CreateAssignOf(Slaves[Index]->read(), SlaveReadRef));
        Parent->addStmt(CreateAssignOf(SlaveReadDataRef, Slaves[Index]->readdata()));
        Parent->addStmt(CreateAssignOf(SlaveReadDataValidRef, Slaves[Index]->readdatavalid()));
      }
    }
    if (Write) {
      SlaveWrite = Parent->CreateTempWire("mem_array_write", 1, NumSlaves);
      SlaveWriteData = Parent->CreateTempWire("mem_array_writedata", DataWidth, NumSlaves);
      SlaveByteEnable = Parent->CreateTempWire("mem_array_byteenable", DataWidth / 8, NumSlaves);
      for (unsigned Index = 0; Index != NumSlaves; ++Index) {
        VerilogSignalRef *SlaveWriteRef = CreateArrayElementOf(SlaveWrite, Index);
        VerilogSignalRef *SlaveWriteDataRef = CreateArrayElementOf(SlaveWriteData, Index);
        VerilogSignalRef *SlaveByteEnableRef = CreateArrayElementOf(SlaveByteEnable, Index);
        Parent->addStmt(CreateAssignOf(Slaves[Index]->write(), SlaveWriteRef));
        Parent->addStmt(CreateAssignOf(Slaves[Index]->writedata(), SlaveWriteDataRef));
        Parent->addStmt(CreateAssignOf(Slaves[Index]->byteenable(), SlaveByteEnableRef));
      }
    }
    SlaveWaitRequest = Parent->CreateTempWire("mem_array_waitrequest", 1, NumSlaves);
    for (unsigned Index = 0; Index != NumSlaves; ++Index) {
      VerilogSignalRef *SlaveWaitRequestRef = CreateArrayElementOf(SlaveWaitRequest, Index);
      Parent->addStmt(CreateAssignOf(SlaveWaitRequestRef, Slaves[Index]->waitrequest()));
    }
    if (Handshakable) {
      SlaveWaitResponse = Parent->CreateTempWire("mem_array_waitresponse", 1, NumSlaves);
      for (unsigned Index = 0; Index != NumSlaves; ++Index) {
        VerilogSignalRef *SlaveWaitResponseRef = CreateArrayElementOf(SlaveWaitResponse, Index);
        Parent->addStmt(CreateAssignOf(Slaves[Index]->waitresponse(), SlaveWaitResponseRef));
      }
    }
  }

  VerilogModuleInstance *Impl;
  if (NumSlaves == 1) {
    if (Read && Write) {
      Impl = Parent->CreateIPInstance("dc_arbiter_n1", "dc_arbiter");
    } else if (Read) {
      Impl = Parent->CreateIPInstance("dc_arbiter_n1_ro", "dc_arbiter");
    } else if (Write) {
      Impl = Parent->CreateIPInstance("dc_arbiter_n1_wo", "dc_arbiter");
    } else {
      llvm_unreachable("impossible case");
    }
  } else {
    if (Read && Write) {
      Impl = Parent->CreateIPInstance("dc_arbiter_nm", "dc_arbiter");
    } else if (Read) {
      Impl = Parent->CreateIPInstance("dc_arbiter_nm_ro", "dc_arbiter");
    } else if (Write) {
      Impl = Parent->CreateIPInstance("dc_arbiter_nm_wo", "dc_arbiter");
    } else {
      llvm_unreachable("impossible case");
    }
  }
  assert(Impl != NULL);
  Impl->addParamArgument("NUM_MASTERS", VerilogParamConst(NumMasters));
  if (NumSlaves > 1) {
    Impl->addParamArgument("NUM_SLAVES", VerilogParamConst(NumSlaves));
  }
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(AddressWidth));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(DataWidth));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef());
  Impl->addArgument("avs_address", new VerilogSignalRef(MasterAddress));
  if (Read) {
    Impl->addArgument("avs_read", new VerilogSignalRef(MasterRead));
    Impl->addArgument("avs_readdata", new VerilogSignalRef(MasterReadData));
    Impl->addArgument("avs_readdatavalid", new VerilogSignalRef(MasterReadDataValid));
  }
  if (Write) {
    Impl->addArgument("avs_write", new VerilogSignalRef(MasterWrite));
    Impl->addArgument("avs_writedata", new VerilogSignalRef(MasterWriteData));
    Impl->addArgument("avs_byteenable", new VerilogSignalRef(MasterByteEnable));
  }
  Impl->addArgument("avs_waitrequest", new VerilogSignalRef(MasterWaitRequest));
  if (Handshakable) {
    Impl->addArgument("avs_waitresponse", new VerilogSignalRef(MasterWaitResponse));
  }
  Impl->addArgument("avm_address", new VerilogSignalRef(SlaveAddress));
  if (Read) {
    Impl->addArgument("avm_read", new VerilogSignalRef(SlaveRead));
    Impl->addArgument("avm_readdata", new VerilogSignalRef(SlaveReadData));
    Impl->addArgument("avm_readdatavalid", new VerilogSignalRef(SlaveReadDataValid));
  }
  if (Write) {
    Impl->addArgument("avm_write", new VerilogSignalRef(SlaveWrite));
    Impl->addArgument("avm_writedata", new VerilogSignalRef(SlaveWriteData));
    Impl->addArgument("avm_byteenable", new VerilogSignalRef(SlaveByteEnable));
  }
  Impl->addArgument("avm_waitrequest", new VerilogSignalRef(SlaveWaitRequest));
  if (Handshakable) {
    Impl->addArgument("avm_waitresponse", new VerilogSignalRef(SlaveWaitResponse));
  }

  L = Slaves;
}

unsigned GlobalMemorySubsystem::ComputeNumRequiredPorts() {
  unsigned Required = 0;
  for (GroupInterfaceListMap::iterator I = Interfaces.begin(),
                                       E = Interfaces.end();
       I != E; ++I) {
    Required += I->second.size();
  }
  return Required;
}

void GlobalMemorySubsystem::SynthesizeMoreAMMPorts(unsigned Required) {
  assert(!AMMPorts.empty());
  while (AMMPorts.size() < Required) {
    AMMInterface *From = AMMPorts.front();
    AMMInterface *To0 = new AMMInterface(Parent, From->getNameAsString() + "_0",
                                         From->getAddressWidth(),
                                         From->getDataWidth(),
                                         From->getBurstWidth());
    AMMInterface *To1 = new AMMInterface(Parent, From->getNameAsString() + "_1",
                                         From->getAddressWidth(),
                                         From->getDataWidth(),
                                         From->getBurstWidth());
    AMMInterface *To2 = new AMMInterface(Parent, From->getNameAsString() + "_2",
                                         From->getAddressWidth(),
                                         From->getDataWidth(),
                                         From->getBurstWidth());
    AMMInterface *To3 = new AMMInterface(Parent, From->getNameAsString() + "_3",
                                         From->getAddressWidth(),
                                         From->getDataWidth(),
                                         From->getBurstWidth());
    VerilogModuleInstance *Impl = Parent->CreateIPInstance("mem_arbiter_41",
                                                           "mem_arbiter_inst");
    assert(Impl != NULL);
    Impl->addArgument("clk", Parent->CreateClockRef());
    Impl->addArgument("rstn", Parent->CreateRstnRef());
    From->ConnectToModuleInstanceAsSlave(Impl, "avm");
    To0->ConnectToModuleInstanceAsMaster(Impl, "avs0");
    To1->ConnectToModuleInstanceAsMaster(Impl, "avs1");
    To2->ConnectToModuleInstanceAsMaster(Impl, "avs2");
    To3->ConnectToModuleInstanceAsMaster(Impl, "avs3");
    AMMPorts.erase(AMMPorts.begin());
    AMMPorts.push_back(To0);
    AMMPorts.push_back(To1);
    AMMPorts.push_back(To2);
    AMMPorts.push_back(To3);
  }
}

void GlobalMemorySubsystem::SynthesizeMoreAXIPorts(unsigned Required) {
  assert(!AXIPorts.empty());
  while (AXIPorts.size() < Required) {
    AXIInterface *From = AXIPorts.front();
    AXIInterface *To0 = new AXIInterface(Parent, From->getNameAsString() + "_0",
                                         From->getAddressWidth(),
                                         From->getDataWidth());
    AXIInterface *To1 = new AXIInterface(Parent, From->getNameAsString() + "_1",
                                         From->getAddressWidth(),
                                         From->getDataWidth());
    AXIInterface *To2 = new AXIInterface(Parent, From->getNameAsString() + "_2",
                                         From->getAddressWidth(),
                                         From->getDataWidth());
    AXIInterface *To3 = new AXIInterface(Parent, From->getNameAsString() + "_3",
                                         From->getAddressWidth(),
                                         From->getDataWidth());
    VerilogModuleInstance *Impl = Parent->CreateIPInstance("mem_arbiter_41",
                                                           "mem_arbiter_inst");
    assert(Impl != NULL);
    Impl->addArgument("clk", Parent->CreateClockRef());
    Impl->addArgument("rstn", Parent->CreateRstnRef());
    From->ConnectToModuleInstance(Impl, "mem0");
    To0->ConnectToModuleInstance(Impl, "mem1");
    To1->ConnectToModuleInstance(Impl, "mem2");
    To2->ConnectToModuleInstance(Impl, "mem3");
    To3->ConnectToModuleInstance(Impl, "mem4");
    AXIPorts.erase(AXIPorts.begin());
    AXIPorts.push_back(To0);
    AXIPorts.push_back(To1);
    AXIPorts.push_back(To2);
    AXIPorts.push_back(To3);
  }
}

void GlobalMemorySubsystem::ConvertAXItoAMMPorts() {
  assert(!AXIPorts.empty());
  for (SmallVectorImpl<AXIInterface*>::iterator I = AXIPorts.begin(),
                                                E = AXIPorts.end();
       I != E; ++I) {
    AXIInterface *From = *I;
    AMMInterface *To = new AMMInterface(Parent, From->getNameAsString() + "_amm",
                                        From->getAddressWidth(),
                                        From->getDataWidth());
    To->ConnectToAXISlave(From);
    AMMPorts.push_back(To);
  }
}

void GlobalMemorySubsystem::SynthesizeCaches(const InterfaceList &L,
                                             VerilogSignal *Clean) {
  CodeGenContext &Ctx = Parent->getCodeGenContext();
  if (Ctx.getMemorySubsystemKind() == SnuCLOptions::MSK_BYPASS) {
    for (InterfaceList::const_iterator I = L.begin(), E = L.end();
         I != E; ++I) {
      SynthesizeBypass(*I, Clean);
    }
  } else {
    for (InterfaceList::const_iterator I = L.begin(), E = L.end();
         I != E; ++I) {
      SynthesizeCache(*I, Clean);
    }
  }
}

void GlobalMemorySubsystem::SynthesizeBypass(AMMInterfaceBase *Master,
                                             VerilogSignal *Clean) {
  AMMInterface *Slave = AMMPorts[NumConsumedPorts++];
  VerilogModuleInstance *Impl = Parent->CreateIPInstance("global_mem_bypass", "bypass");
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(Master->getAddressWidth()));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(Slave->getDataWidth()));
  Impl->addParamArgument("DATAPATH_DATA_WIDTH", VerilogParamConst(Master->getDataWidth()));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef("opencl_rstn"));
  Slave->ConnectToModuleInstanceAsSlave(Impl, "avm");
  Master->ConnectToModuleInstanceAsMaster(Impl, "avs");
}

void GlobalMemorySubsystem::SynthesizeCache(AMMInterfaceBase *Master,
                                            VerilogSignal *Clean) {
  AMMInterface *Slave = AMMPorts[NumConsumedPorts++];
  VerilogModuleInstance *Impl;
  if (Master->isReadable() && Master->isWritable()) {
    Impl = Parent->CreateIPInstance("global_mem_cache", "cache");
  } else if (Master->isReadable()) {
    Impl = Parent->CreateIPInstance("global_mem_cache_ro", "cache");
  } else if (Master->isWritable()) {
    Impl = Parent->CreateIPInstance("global_mem_cache_wo", "cache");
  } else {
    llvm_unreachable("impossible");
  }
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(Master->getAddressWidth()));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(Slave->getDataWidth()));
  Impl->addParamArgument("DATAPATH_DATA_WIDTH", VerilogParamConst(Master->getDataWidth()));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef("opencl_rstn"));
  Slave->ConnectToModuleInstanceAsSlave(Impl, "avm");
  Master->ConnectToModuleInstanceAsMaster(Impl, "avs");

  VerilogSignal *Cleaned = Parent->CreateTempWire("cache_cleaned");
  SubCleaned.push_back(Cleaned);
  Impl->addArgument("cache_clean", new VerilogSignalRef(Clean));
  Impl->addArgument("cache_cleaned", new VerilogSignalRef(Cleaned));
}

void GlobalMemorySubsystem::SynthesizeCleaned(VerilogSignal *GlobalCleaned) {
  if (SubCleaned.empty()) {
    Parent->addAssignWire(GlobalCleaned, CreateOne());
  } else {
    VerilogExpr *GlobalCleanedExpr = NULL;
    for (SmallVectorImpl<VerilogSignal*>::iterator I = SubCleaned.begin(),
                                                   E = SubCleaned.end();
         I != E; ++I) {
      VerilogSignal *Cleaned = *I;
      VerilogSignal *CleanedR = Parent->CreateTempReg("cache_cleaned_r");
      Parent->addAssignRegStmt(CleanedR,
        // if (global_cleaned)
        new VerilogIf(new VerilogSignalRef(GlobalCleaned),
          // begin
          (new VerilogCompound())->addStmt(
            // cache_cleaned_r <= 1'b0;
            CreateAssignOf(CleanedR, CreateZero())
          ),
          // end
          // else if (cache_cleaned)
          new VerilogIf(new VerilogSignalRef(Cleaned),
            // begin
            (new VerilogCompound())->addStmt(
              // cache_cleaned_r <= 1'b1;
              CreateAssignOf(CleanedR, CreateOne())
            )
            // end
          )
        )
      );
      if (GlobalCleanedExpr == NULL) {
        GlobalCleanedExpr = new VerilogSignalRef(CleanedR);
      } else {
        GlobalCleanedExpr = CreateAndOf(GlobalCleanedExpr, CleanedR);
      }
    }
    assert(GlobalCleanedExpr != NULL);
    Parent->addAssignWire(GlobalCleaned, GlobalCleanedExpr);
  }
}

// Local Memory Subsystem

LocalMemoryLayout::LocalMemoryLayout(CodeGenContext &ctx,
                                     const ControlDataflowGraph *program,
                                     const AliasSet &aliases)
  : Ctx(ctx), Program(program), Aliases(aliases) {
  NumLayers = Ctx.getTopLevelSingleWorkGroupGranularity();
  LayerWidth = Log2(NumLayers);
  for (ControlDataflowGraph::const_iterator G = Program->begin(),
                                            GEnd = Program->end();
       G != GEnd; ++G) {
    if (DataflowGraph *Graph = *G) {
      for (DataflowGraph::iterator N = Graph->begin(), NEnd = Graph->end();
           N != NEnd; ++N) {
        if (DFGAddrOfNode *AddrOf = dyn_cast<DFGAddrOfNode>(*N)) {
          Allocate(AddrOf->getVariable());
        }
      }
    }
  }
  DataflowGraph *Entry = Program->getEntry();
  for (DataflowGraph::const_live_var_iterator L = Entry->live_in_begin(),
                                              LEnd = Entry->live_in_end();
       L != LEnd; ++L) {
    IndexedVarDecl *LiveIn = *L;
    if (LiveIn->isParameter() && LiveIn->getType()->isPointerType() &&
        LiveIn->getType()->getPointeeType().getAddressSpace() == LangAS::opencl_local) {
      Allocate(LiveIn->getDecl(), Ctx.getCapacityLocalMemChunk());
    }
  }
  for (ControlDataflowGraph::const_iterator G = Program->begin(),
                                            GEnd = Program->end();
       G != GEnd; ++G) {
    if (DataflowGraph *Graph = *G) {
      for (DataflowGraph::iterator N = Graph->begin(), NEnd = Graph->end();
           N != NEnd; ++N) {
        if (DFGMemoryAccessNode *Access = dyn_cast<DFGMemoryAccessNode>(*N)) {
          if (Access->getAddressSpace() == LangAS::opencl_local) {
            UpdateGranularity(Access->getAliasGroup(), Access->getAccessType());
          }
        }
      }
    }
  }
}

void LocalMemoryLayout::Allocate(WVarDecl *Var, uint64_t AllocSize) {
  if (Offset.count(Var) || !Aliases.isUsed(Var)) {
    return;
  }
  if (AllocSize == 0) {
    AllocSize = Ctx.getTypeSizeInChars(Var);
    if (Var->getType().getAddressSpace() == 0) {
      AllocSize *= (1 << Ctx.getPortLIDWidth());
    }
  }
  AllocSize = (AllocSize + 63) / 64 * 64;
  unsigned Alias = Aliases.getAlias(Var);
  if (!Size.count(Alias)) {
    Size[Alias] = 0;
    Granularity[Alias] = 1;
  }
  Offset[Var] = Size[Alias];
  Size[Alias] += AllocSize;
}

void LocalMemoryLayout::UpdateGranularity(unsigned Alias, QualType AccessType) {
  assert(Granularity.count(Alias));
  unsigned AccessWidth = Ctx.getTypeSizeInChars(AccessType);
  while (Granularity[Alias] < AccessWidth) {
    Granularity[Alias] *= 2;
  }
  assert(Granularity[Alias] <= 64);
  assert(Size.count(Alias) && Granularity[Alias] <= Size[Alias]);
}

uint64_t LocalMemoryLayout::getAddressOf(WVarDecl *Var) const {
  return Offset.count(Var) ? Offset.lookup(Var) : 0;
}

unsigned LocalMemoryLayout::getVirtualAddressWidthOf(unsigned Alias) const {
  assert(Size.count(Alias));
  return Log2(Size.lookup(Alias) - 1) + 1;
}

unsigned LocalMemoryLayout::getPhysicalAddressWidthOf(unsigned Alias) const {
  return getVirtualAddressWidthOf(Alias) + LayerWidth;
}

unsigned LocalMemoryLayout::getDataWidthOf(unsigned Alias) const {
  assert(Granularity.count(Alias));
  return Granularity.lookup(Alias) * 8;
}

unsigned LocalMemoryLayout::getVirtualAddressWidthFor(
    DFGMemoryAccessNode *Access) const {
  assert(Access->getAddressSpace() == LangAS::opencl_local);
  return getVirtualAddressWidthOf(Access->getAliasGroup());
}

unsigned LocalMemoryLayout::getPhysicalAddressWidthFor(
    DFGMemoryAccessNode *Access) const {
  assert(Access->getAddressSpace() == LangAS::opencl_local);
  return getPhysicalAddressWidthOf(Access->getAliasGroup());
}

unsigned LocalMemoryLayout::getDataWidthFor(DFGMemoryAccessNode *Access) const {
  assert(Access->getAddressSpace() == LangAS::opencl_local);
  return getDataWidthOf(Access->getAliasGroup());
}

LocalMemorySubsystem::LocalMemorySubsystem(ModuleBuilder *parent,
                                           const LocalMemoryLayout &layout)
  : Parent(parent), Layout(layout), IsFinalized(false) {
}

void LocalMemorySubsystem::addInterface(AMMInterfaceBase *I) {
  assert(!IsFinalized);
  assert(I->parent() == Parent);
  assert(I->getAddressSpace() == LangAS::opencl_local);
  Interfaces[I->getAliasGroup()].push_back(I);
}

void LocalMemorySubsystem::Finalize() {
  assert(!IsFinalized);
  IsFinalized = true;

  for (GroupInterfaceListMap::iterator I = Interfaces.begin(),
                                       E = Interfaces.end();
       I != E; ++I) {
    SynthesizeLocalMemory(I->first, I->second);
  }
}

void LocalMemorySubsystem::SynthesizeLocalMemory(unsigned Alias,
                                                 const InterfaceList &L) {
  assert(!L.empty());
  unsigned NumMasters = L.size();
  InterfaceList Masters = L;
  unsigned AddressWidth = Layout.getPhysicalAddressWidthOf(Alias);
  unsigned DataWidth = Layout.getDataWidthOf(Alias);

  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    if (Masters[Index]->getDataWidth() != DataWidth) {
      AMMInterfaceBase *NewMaster = Masters[Index]->ResizedClone(
          Parent, Parent->AcquireNameWithPrefix("mem_resized"), DataWidth);
      Masters[Index]->ConnectToSlave(NewMaster);
      Masters[Index] = NewMaster;
    }
  }

  VerilogSignal *MasterAddress = Parent->CreateTempWire("mem_array_address", AddressWidth, NumMasters);
  VerilogSignal *MasterRead = Parent->CreateTempWire("mem_array_read", 1, NumMasters);
  VerilogSignal *MasterReadData = Parent->CreateTempWire("mem_array_readdata", DataWidth, NumMasters);
  VerilogSignal *MasterReadDataValid = Parent->CreateTempWire("mem_array_readdatavalid", 1, NumMasters);
  VerilogSignal *MasterWrite = Parent->CreateTempWire("mem_array_write", 1, NumMasters);
  VerilogSignal *MasterWriteData = Parent->CreateTempWire("mem_array_writedata", DataWidth, NumMasters);
  VerilogSignal *MasterByteEnable = Parent->CreateTempWire("mem_array_byteenable", DataWidth / 8, NumMasters);
  VerilogSignal *MasterWaitRequest = Parent->CreateTempWire("mem_array_waitrequest", 1, NumMasters);
  VerilogSignal *MasterWaitResponse = Parent->CreateTempWire("mem_array_waitresponse", 1, NumMasters);
  for (unsigned Index = 0; Index != NumMasters; ++Index) {
    VerilogSignalRef *MasterAddressRef = CreateArrayElementOf(MasterAddress, Index);
    Parent->addStmt(CreateAssignOf(MasterAddressRef, Masters[Index]->address()));
    if (Masters[Index]->isReadable()) {
      VerilogSignalRef *MasterReadRef = CreateArrayElementOf(MasterRead, Index);
      VerilogSignalRef *MasterReadDataRef = CreateArrayElementOf(MasterReadData, Index);
      VerilogSignalRef *MasterReadDataValidRef = CreateArrayElementOf(MasterReadDataValid, Index);
      Parent->addStmt(CreateAssignOf(MasterReadRef, Masters[Index]->read()));
      Parent->addStmt(CreateAssignOf(Masters[Index]->readdata(), MasterReadDataRef));
      Parent->addStmt(CreateAssignOf(Masters[Index]->readdatavalid(), MasterReadDataValidRef));
    } else {
      VerilogSignalRef *MasterReadRef = CreateArrayElementOf(MasterRead, Index);
      Parent->addStmt(CreateAssignOf(MasterReadRef, CreateZero()));
    }
    VerilogSignalRef *MasterWriteRef = CreateArrayElementOf(MasterWrite, Index);
    VerilogSignalRef *MasterWriteDataRef = CreateArrayElementOf(MasterWriteData, Index);
    VerilogSignalRef *MasterByteEnableRef = CreateArrayElementOf(MasterByteEnable, Index);
    if (Masters[Index]->isWritable()) {
      Parent->addStmt(CreateAssignOf(MasterWriteRef, Masters[Index]->write()));
      Parent->addStmt(CreateAssignOf(MasterWriteDataRef, Masters[Index]->writedata()));
      Parent->addStmt(CreateAssignOf(MasterByteEnableRef, Masters[Index]->byteenable()));
    } else {
      Parent->addStmt(CreateAssignOf(MasterWriteRef, CreateZero()));
      Parent->addStmt(CreateAssignOf(MasterWriteDataRef, CreateZero(DataWidth)));
      Parent->addStmt(CreateAssignOf(MasterByteEnableRef, CreateZero(DataWidth / 8)));
    }
    VerilogSignalRef *MasterWaitRequestRef = CreateArrayElementOf(MasterWaitRequest, Index);
    Parent->addStmt(CreateAssignOf(Masters[Index]->waitrequest(), MasterWaitRequestRef));
    VerilogSignalRef *MasterWaitResponseRef = CreateArrayElementOf(MasterWaitResponse, Index);
    if (Masters[Index]->isHandshakable()) {
      Parent->addStmt(CreateAssignOf(MasterWaitResponseRef, Masters[Index]->waitresponse()));
    } else {
      Parent->addStmt(CreateAssignOf(MasterWaitResponseRef, CreateZero()));
    }
  }

  VerilogModuleInstance *Impl = Parent->CreateIPInstance("local_memory", "lmem");
  Impl->addParamArgument("NUM_MASTERS", VerilogParamConst(NumMasters));
  Impl->addParamArgument("ADDR_WIDTH", VerilogParamConst(AddressWidth));
  Impl->addParamArgument("DATA_WIDTH", VerilogParamConst(DataWidth));
  Impl->addParamArgument("BANK_WIDTH", VerilogParamConst(Log2(NumMasters) + 1));
  Impl->addArgument("clk", Parent->CreateClockRef());
  Impl->addArgument("rstn", Parent->CreateRstnRef("opencl_rstn"));
  Impl->addArgument("avs_address", new VerilogSignalRef(MasterAddress));
  Impl->addArgument("avs_read", new VerilogSignalRef(MasterRead));
  Impl->addArgument("avs_readdata", new VerilogSignalRef(MasterReadData));
  Impl->addArgument("avs_readdatavalid", new VerilogSignalRef(MasterReadDataValid));
  Impl->addArgument("avs_write", new VerilogSignalRef(MasterWrite));
  Impl->addArgument("avs_writedata", new VerilogSignalRef(MasterWriteData));
  Impl->addArgument("avs_byteenable", new VerilogSignalRef(MasterByteEnable));
  Impl->addArgument("avs_waitrequest", new VerilogSignalRef(MasterWaitRequest));
  Impl->addArgument("avs_waitresponse", new VerilogSignalRef(MasterWaitResponse));
}

} // namespace Synthesis

} // namespace snu

} // namespace clang
