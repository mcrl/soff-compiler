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

#include "clang/SnuSynthesis/Verilog.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>
#include <sstream>
#include <string>

namespace clang {

namespace snu {

// VerilogSignal

VerilogSignal::VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                             VerilogSignalPortKind portKind, StringRef name,
                             bool is_signed)
  : Parent(parent), Kind(kind), PortKind(portKind), Attr(SA_None), Name(name),
    VectorWidth(1), ArrayWidth(0), Signed(is_signed) {
}

VerilogSignal::VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                             VerilogSignalPortKind portKind, StringRef name,
                             VerilogParamConst vectorWidth, bool is_signed)
  : Parent(parent), Kind(kind), PortKind(portKind), Attr(SA_None), Name(name),
    VectorWidth(vectorWidth), ArrayWidth(0), Signed(is_signed) {
  assert(VectorWidth.CanBeGreaterThan(0));
}

VerilogSignal::VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                             VerilogSignalPortKind portKind, StringRef name,
                             VerilogParamConst vectorWidth,
                             VerilogParamConst arrayWidth, bool is_signed)
  : Parent(parent), Kind(kind), PortKind(portKind), Attr(SA_None), Name(name),
    VectorWidth(vectorWidth), ArrayWidth(arrayWidth), Signed(is_signed) {
  assert(VectorWidth.CanBeGreaterThan(0));
  assert(ArrayWidth.CanBeGreaterThan(0));
}

bool VerilogSignal::isVector() const {
  return VectorWidth.CanBeGreaterThan(1);
}

unsigned VerilogSignal::getVectorWidthValue() const {
  assert(!VectorWidth.isParametrized());
  return (unsigned)VectorWidth.getValue();
}

bool VerilogSignal::isArray() const {
  return ArrayWidth.CanBeGreaterThan(0);
}

unsigned VerilogSignal::getArrayWidthValue() const {
  assert(!ArrayWidth.isParametrized());
  return (unsigned)ArrayWidth.getValue();
}

// VerilogModule

bool VerilogModule::hasArgument(StringRef name) const {
  VerilogSignal *S = FindSignal(name);
  if (S == NULL) return false;
  return S->isPort();
}

void VerilogModule::addParameter(VerilogParameter *param) {
  assert(FindParameter(param->getName()) == NULL);
  Params.push_back(param);
}

void VerilogModule::addSignal(VerilogSignal *signal) {
  assert(FindSignal(signal->getName()) == NULL);
  Signals.push_back(signal);
  SignalMap[signal->getName()] = signal;
}

void VerilogModule::addModuleInstance(VerilogModuleInstance *inst) {
  ModuleInsts.push_back(inst);
}

void VerilogModule::addStmt(VerilogStmt *S) {
  Body.push_back(S);
  if (VerilogAssign *Assign = dyn_cast<VerilogAssign>(S)) {
    Assign->setAsTopLevel();
  }
}

VerilogParameter *VerilogModule::CreateParameter(VerilogParameterKind kind,
                                                 StringRef name,
                                                 int defaultValue) {
  VerilogParameter *param = new VerilogParameter(this, kind, name,
                                                 defaultValue);
  addParameter(param);
  return param;
}

VerilogSignal *VerilogModule::CreateWire(VerilogSignalPortKind portKind,
                                         StringRef name) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, portKind, name);
  addSignal(signal);
  return signal;
}

VerilogSignal *VerilogModule::CreateWire(VerilogSignalPortKind portKind,
                                         StringRef name,
                                         VerilogParamConst vectorWidth) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, portKind, name,
                                            vectorWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *VerilogModule::CreateWire(VerilogSignalPortKind portKind,
                                         StringRef name,
                                         VerilogParamConst vectorWidth,
                                         VerilogParamConst arrayWidth) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Wire, portKind, name,
                                            vectorWidth, arrayWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *VerilogModule::CreateReg(VerilogSignalPortKind portKind,
                                        StringRef name) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, portKind, name);
  addSignal(signal);
  return signal;
}

VerilogSignal *VerilogModule::CreateReg(VerilogSignalPortKind portKind,
                                        StringRef name,
                                        VerilogParamConst vectorWidth) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, portKind, name,
                                            vectorWidth);
  addSignal(signal);
  return signal;
}

VerilogSignal *VerilogModule::CreateReg(VerilogSignalPortKind portKind,
                                        StringRef name,
                                        VerilogParamConst vectorWidth,
                                        VerilogParamConst arrayWidth) {
  VerilogSignal *signal = new VerilogSignal(this, SK_Reg, portKind, name,
                                            vectorWidth, arrayWidth);
  addSignal(signal);
  return signal;
}

VerilogModuleInstance *VerilogModule::CreateModuleInstance(
    VerilogModule *target, StringRef name) {
  VerilogModuleInstance *inst = new VerilogModuleInstance(this, target, name);
  addModuleInstance(inst);
  return inst;
}

VerilogParameter *VerilogModule::FindParameter(StringRef name) const {
  for (const_param_iterator P = param_begin(), PEnd = param_end();
       P != PEnd; ++P) {
    if ((*P)->getName() == name)
      return *P;
  }
  return NULL;
}

VerilogSignal *VerilogModule::FindSignal(StringRef name) const {
  if (SignalMap.count(name)) {
    return SignalMap.lookup(name);
  } else {
    return NULL;
  }
}

// VerilogModuleInstance

void VerilogModuleInstance::addParamArgument(VerilogParameter *param,
                                             VerilogParamConst value) {
  assert(param->getParent() == getTarget());
  assert(!ParamArgSet.count(param));
  ParamArgs.push_back(ParamArgPair(param, value));
  ParamArgSet.insert(param);
}

void VerilogModuleInstance::addParamArgument(StringRef paramName,
                                             VerilogParamConst value) {
  addParamArgument(Target->getParameter(paramName), value);
}

void VerilogModuleInstance::addArgument(VerilogSignal *arg,
                                        VerilogExpr *value) {
  assert(arg->getParent() == getTarget());
  assert(arg->isPort());
  assert(!ArgSet.count(arg));
  Args.push_back(ArgPair(arg, value));
  ArgSet.insert(arg);
}

void VerilogModuleInstance::addArgument(StringRef argName,
                                        VerilogExpr *value) {
  addArgument(Target->getSignal(argName), value);
}

// VerilogConst

namespace {

bool IsValidDigit(char c, VerilogValueRadixKind radix) {
  if (c == 'x' || c == 'X' || c == 'z' || c == 'Z') return true;
  switch (radix) {
    case VR_Binary:  return (c >= '0' && c <= '1');
    case VR_Octal:   return (c >= '0' && c <= '1');
    case VR_Decimal: return (c >= '0' && c <= '9');
    case VR_Hexa:    return (c >= '0' && c <= '9') ||
                            (c >= 'A' && c <= 'F') ||
                            (c >= 'a' && c <= 'f');
    default: llvm_unreachable("invalid");
  }
}

} // anonymous namespace

VerilogConst::VerilogConst(VerilogValueRadixKind radix, StringRef value,
                           unsigned size)
  : VerilogExpr(VS_ConstClass), RadixKind(radix), Value(value), Size(size) {
  for (unsigned Index = 0, NumDigits = Value.size();
       Index != NumDigits; ++Index) {
    assert(IsValidDigit(Value[Index], RadixKind));
  }
}

VerilogConst::VerilogConst(VerilogValueRadixKind radix, uint64_t value,
                           unsigned size)
  : VerilogExpr(VS_ConstClass), RadixKind(radix), Size(size) {
  assert(!(Size > 64 && value != 0));

  unsigned Radix;
  switch (RadixKind) {
    case VR_Binary:  Radix = 2; break;
    case VR_Octal:   Radix = 8; break;
    case VR_Decimal: Radix = 10; break;
    case VR_Hexa:    Radix = 16; break;
    default: llvm_unreachable("impossible");
  }

  if (RadixKind == VR_Decimal || Size == 0) {
    // no padding
    while (value > 0) {
      unsigned digit = value % Radix;
      value /= Radix;
      Value.push_back((char)(digit < 10 ? digit + '0' : digit - 10 + 'A'));
    }
    if (Value.empty()) {
      Value.push_back('0');
    }
  } else {
    // zero-padding
    unsigned NumDigits;
    switch (RadixKind) {
      case VR_Binary: NumDigits = Size; break;
      case VR_Octal:  NumDigits = (Size + 2) / 3; break;
      case VR_Hexa:   NumDigits = (Size + 3) / 4; break;
      default: llvm_unreachable("impossible");
    }
    while (NumDigits > 0) {
      unsigned digit = value % Radix;
      value /= Radix;
      Value.push_back((char)(digit < 10 ? digit + '0' : digit - 10 + 'A'));
      NumDigits--;
    }
  }
  std::reverse(Value.begin(), Value.end());
}

char VerilogConst::getRadixCode() const {
  switch (RadixKind) {
    case VR_Binary:  return 'b';
    case VR_Octal:   return 'o';
    case VR_Decimal: return 'd';
    case VR_Hexa:    return 'h';
    default: llvm_unreachable("impossible");
  }
}

std::string VerilogConst::getAsString() const {
  std::ostringstream out;
  if (isSized()) {
    out << Size << '\'' << getRadixCode();
  } else if (RadixKind != VR_Decimal) {
    out << '\'' << getRadixCode();
  }
  out << Value;
  return out.str();
}

// VerilogSignalRef

VerilogSignalRef::VerilogSignalRef(VerilogSignal *signal)
  : VerilogExpr(VS_SignalRefClass), Signal(signal), ArrayIndex(NULL),
    MSB(signal->getVectorWidth(), -1), LSB(0) {
}

VerilogSignalRef::VerilogSignalRef(VerilogSignal *signal, VerilogParamConst msb,
                                   VerilogParamConst lsb)
  : VerilogExpr(VS_SignalRefClass), Signal(signal), ArrayIndex(NULL),
    MSB(msb), LSB(lsb) {
  assert(signal->isVector());
}

VerilogSignalRef::VerilogSignalRef(VerilogSignal *signal, VerilogExpr *index)
  : VerilogExpr(VS_SignalRefClass), Signal(signal), ArrayIndex(index),
    MSB(signal->getVectorWidth(), -1), LSB(0) {
  assert(signal->isArray());
}

VerilogSignalRef::VerilogSignalRef(VerilogSignal *signal, VerilogExpr *index,
                                   VerilogParamConst msb, VerilogParamConst lsb)
  : VerilogExpr(VS_SignalRefClass), Signal(signal), ArrayIndex(index),
    MSB(msb), LSB(lsb) {
  assert(signal->isVector());
  assert(signal->isArray());
}

bool VerilogSignalRef::hasVectorRange() const {
  return MSB != VerilogParamConst(Signal->getVectorWidth(), -1) ||
         LSB != VerilogParamConst(0);
}

// VerilogPartSelect

VerilogPartSelect::VerilogPartSelect(VerilogSignal *signal, VerilogExpr *offset,
                                     VerilogParamConst width)
  : VerilogExpr(VS_PartSelectClass), Signal(signal), Offset(offset),
    Width(width) {
  assert(signal->isVector());
}

} // namespace snu

} // namespace clang
