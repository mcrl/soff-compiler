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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_VERILOG_H
#define LLVM_CLANG_SNU_SYNTHESIS_VERILOG_H

#include "clang/Basic/LLVM.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include <string>
#include <utility>

namespace clang {

namespace snu {

class VerilogCompound;
class VerilogDefine;
class VerilogEvent;
class VerilogExpr;
class VerilogModule;
class VerilogModuleInstance;
class VerilogParamConst;
class VerilogParameter;
class VerilogSignal;
class VerilogSignalRef;
class VerilogStmt;

// A top-level object

class VerilogFile {
  std::string Name;
  SmallVector<VerilogDefine*, 8> Defines;
  SmallVector<VerilogModule*, 32> Modules;

public:
  explicit VerilogFile(StringRef name)
    : Name(name) {}

  StringRef getName() const { return Name; }

  typedef SmallVectorImpl<VerilogDefine*>::iterator define_iterator;
  typedef SmallVectorImpl<VerilogDefine*>::const_iterator const_define_iterator;

  define_iterator define_begin() { return Defines.begin(); }
  define_iterator define_end() { return Defines.end(); }
  const_define_iterator define_begin() const { return Defines.begin(); }
  const_define_iterator define_end() const { return Defines.end(); }
  unsigned num_defines() const { return Defines.size(); }

  typedef SmallVectorImpl<VerilogModule*>::iterator module_iterator;
  typedef SmallVectorImpl<VerilogModule*>::const_iterator const_module_iterator;

  module_iterator module_begin() { return Modules.begin(); }
  module_iterator module_end() { return Modules.end(); }
  const_module_iterator module_begin() const { return Modules.begin(); }
  const_module_iterator module_end() const { return Modules.end(); }
  unsigned num_modules() const { return Modules.size(); }

  void addDefine(VerilogDefine *define) {
    Defines.push_back(define);
  }
  void addModule(VerilogModule *module) {
    Modules.push_back(module);
  }

  void print(raw_ostream &OS) const;
};

// Parameters and parametrized constants

enum VerilogParameterKind {
  PK_Param,
  PK_LocalParam
};

class VerilogParameter {
  VerilogModule *Parent;
  VerilogParameterKind Kind;
  std::string Name;
  int DefaultValue;

public:
  VerilogParameter(VerilogModule *parent, VerilogParameterKind kind,
                   StringRef name, int defaultValue)
    : Parent(parent), Kind(kind), Name(name), DefaultValue(defaultValue) {}

  VerilogModule *getParent() const { return Parent; }
  VerilogParameterKind getKind() const { return Kind; }
  StringRef getName() const { return Name; }
  int getDefaultValue() const { return DefaultValue; }

  void print(raw_ostream &OS) const;
};

class VerilogParamConst {
  VerilogParameter *Param;
  int ParamDelta;
  int Value;

public:
  VerilogParamConst()
    : Param(NULL), ParamDelta(0), Value(0) {}
  // param
  explicit VerilogParamConst(VerilogParameter *param)
    : Param(param), ParamDelta(0), Value(0) {}
  // param + delta
  VerilogParamConst(VerilogParameter *param, int delta)
    : Param(param), ParamDelta(delta), Value(0) {}
  // value
  explicit VerilogParamConst(int value)
    : Param(NULL), ParamDelta(0), Value(value) {}
  // C
  VerilogParamConst(const VerilogParamConst &C)
    : Param(C.Param), ParamDelta(C.ParamDelta), Value(C.Value) {}
  // C + delta
  VerilogParamConst(const VerilogParamConst &C, int delta)
    : Param(C.Param), ParamDelta(C.ParamDelta), Value(C.Value) {
    if (Param != NULL) {
      ParamDelta += delta;
    } else {
      Value += delta;
    }
  }

  bool isParametrized() const { return Param != NULL; }
  VerilogParameter *getParameter() const {
    assert(Param != NULL);
    return Param;
  }
  int getParameterDelta() const {
    assert(Param != NULL);
    return ParamDelta;
  }
  int getValue() const {
    assert(Param == NULL);
    return Value;
  }

  bool operator==(const VerilogParamConst &rhs) const {
    return (Param == rhs.Param && ParamDelta == rhs.ParamDelta &&
            Value == rhs.Value);
  }
  bool operator!=(const VerilogParamConst &rhs) const {
    return (Param != rhs.Param || ParamDelta != rhs.ParamDelta ||
            Value != rhs.Value);
  }
  bool CanBeGreaterThan(int rhs) const {
    return isParametrized() || Value > rhs;
  }

  void print(raw_ostream &OS) const;
};

// Defines, nets and registers

class VerilogDefine {
  std::string Name;
  VerilogExpr *Value;

public:
  VerilogDefine(StringRef name, VerilogExpr *value)
    : Name(name), Value(value) {}

  StringRef getName() const { return Name; }
  VerilogExpr *getValue() const { return Value; }

  void print(raw_ostream &OS) const;
};

enum VerilogSignalKind {
  SK_Wire,
  SK_Reg
};

enum VerilogSignalPortKind {
  SP_None,
  SP_Input,
  SP_Output
};

enum VerilogSignalAttribute {
  SA_None = 0x0,
  SA_DontMerge = 0x1, // Intel (* dont_merge *)
  SA_DontTouch = 0x2 // Xilinx (* dont_touch = "yes" *)
};

class VerilogSignal {
  VerilogModule *Parent;
  VerilogSignalKind Kind;
  VerilogSignalPortKind PortKind;
  unsigned Attr;
  std::string Name;
  VerilogParamConst VectorWidth;
  VerilogParamConst ArrayWidth;
  bool Signed;

public:
  VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                VerilogSignalPortKind portKind, StringRef name,
                bool is_signed = false);
  VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                VerilogSignalPortKind portKind, StringRef name,
                VerilogParamConst vectorWidth, bool is_signed = false);
  VerilogSignal(VerilogModule *parent, VerilogSignalKind kind,
                VerilogSignalPortKind portKind, StringRef name,
                VerilogParamConst vectorWidth, VerilogParamConst arrayWidth,
                bool is_signed = false);

  VerilogModule *getParent() const { return Parent; }
  VerilogSignalKind getKind() const { return Kind; }
  VerilogSignalPortKind getPortKind() const { return PortKind; }
  bool isPort() const { return PortKind != SP_None; }
  StringRef getName() const { return Name; }
  std::string getNameAsString() const { return Name; }

  bool isVector() const;
  VerilogParamConst getVectorWidth() const { return VectorWidth; }
  unsigned getVectorWidthValue() const;

  bool isArray() const;
  VerilogParamConst getArrayWidth() const { return ArrayWidth; }
  unsigned getArrayWidthValue() const;

  bool isSigned() const { return Signed; }

  bool hasAnyAttribute() const { return Attr != SA_None; }
  bool hasAttribute(VerilogSignalAttribute A) const { return Attr & A; }
  void addAttribute(VerilogSignalAttribute A) { Attr |= A; }

  void print(raw_ostream &OS) const;
};

// Modules and module instances

class VerilogModule {
  std::string Name;
  SmallVector<VerilogParameter*, 8> Params;
  SmallVector<VerilogSignal*, 64> Signals;
  SmallVector<VerilogModuleInstance*, 8> ModuleInsts;
  SmallVector<VerilogStmt*, 8> Body;
  llvm::StringMap<VerilogSignal*> SignalMap;

public:
  explicit VerilogModule(StringRef name)
    : Name(name.str()) {}

  StringRef getName() const { return Name; }
  std::string getNameAsString() const { return Name; }

  typedef SmallVectorImpl<VerilogParameter*>::iterator param_iterator;
  typedef SmallVectorImpl<VerilogParameter*>::const_iterator
      const_param_iterator;

  param_iterator param_begin() { return Params.begin(); }
  param_iterator param_end() { return Params.end(); }
  const_param_iterator param_begin() const { return Params.begin(); }
  const_param_iterator param_end() const { return Params.end(); }
  unsigned num_params() const { return Params.size(); }

  typedef SmallVectorImpl<VerilogSignal*>::iterator signal_iterator;
  typedef SmallVectorImpl<VerilogSignal*>::const_iterator
      const_signal_iterator;

  signal_iterator signal_begin() { return Signals.begin(); }
  signal_iterator signal_end() { return Signals.end(); }
  const_signal_iterator signal_begin() const { return Signals.begin(); }
  const_signal_iterator signal_end() const { return Signals.end(); }
  unsigned num_signals() const { return Signals.size(); }

  typedef SmallVectorImpl<VerilogModuleInstance*>::iterator
      module_inst_iterator;
  typedef SmallVectorImpl<VerilogModuleInstance*>::const_iterator
      const_module_inst_iterator;

  module_inst_iterator module_inst_begin() { return ModuleInsts.begin(); }
  module_inst_iterator module_inst_end() { return ModuleInsts.end(); }
  const_module_inst_iterator module_inst_begin() const {
    return ModuleInsts.begin();
  }
  const_module_inst_iterator module_inst_end() const {
    return ModuleInsts.end();
  }
  unsigned num_module_insts() const { return ModuleInsts.size(); }

  typedef SmallVectorImpl<VerilogStmt*>::iterator body_iterator;
  typedef SmallVectorImpl<VerilogStmt*>::const_iterator const_body_iterator;

  body_iterator body_begin() { return Body.begin(); }
  body_iterator body_end() { return Body.end(); }
  const_body_iterator body_begin() const { return Body.begin(); }
  const_body_iterator body_end() const { return Body.end(); }
  unsigned body_size() const { return Body.size(); }

  bool hasParameter(StringRef name) const {
    return FindParameter(name) != NULL;
  }
  bool hasSignal(StringRef name) const {
    return FindSignal(name) != NULL;
  }
  bool hasArgument(StringRef name) const;

  VerilogParameter *getParameter(StringRef name) const {
    VerilogParameter *P = FindParameter(name);
    assert(P != NULL && "parameter not found");
    return P;
  }
  VerilogSignal *getSignal(StringRef name) const {
    VerilogSignal *S = FindSignal(name);
    assert(S != NULL && "signal not found");
    return S;
  }

  void addParameter(VerilogParameter *param);
  void addSignal(VerilogSignal *signal);
  void addModuleInstance(VerilogModuleInstance *inst);
  void addStmt(VerilogStmt *S);

  VerilogParameter *CreateParameter(VerilogParameterKind kind, StringRef name,
                                    int defaultValue);

  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name);
  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name,
                            VerilogParamConst vectorWidth);
  VerilogSignal *CreateWire(VerilogSignalPortKind portKind, StringRef name,
                            VerilogParamConst vectorWidth,
                            VerilogParamConst arrayWidth);
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name);
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name,
                           VerilogParamConst vectorWidth);
  VerilogSignal *CreateReg(VerilogSignalPortKind portKind, StringRef name,
                           VerilogParamConst vectorWidth,
                           VerilogParamConst arrayWidth);

  VerilogModuleInstance *CreateModuleInstance(VerilogModule *target,
                                              StringRef name);

  void print(raw_ostream &OS) const;

private:
  VerilogParameter *FindParameter(StringRef name) const;
  VerilogSignal *FindSignal(StringRef name) const;
};

class VerilogModuleInstance {
  VerilogModule *Parent;
  VerilogModule *Target;
  std::string Name;

  typedef std::pair<VerilogParameter*, VerilogParamConst> ParamArgPair;
  typedef std::pair<VerilogSignal*, VerilogExpr*> ArgPair;

  SmallVector<ParamArgPair, 8> ParamArgs;
  SmallVector<ArgPair, 64> Args;
  llvm::DenseSet<VerilogParameter*> ParamArgSet;
  llvm::DenseSet<VerilogSignal*> ArgSet;

public:
  VerilogModuleInstance(VerilogModule *parent, VerilogModule *target,
                        StringRef name)
    : Parent(parent), Target(target), Name(name) {}

  VerilogModule *getParent() const { return Parent; }
  VerilogModule *getTarget() const { return Target; }
  StringRef getName() const { return Name; }

  typedef SmallVectorImpl<ParamArgPair>::iterator param_arg_iterator;
  typedef SmallVectorImpl<ParamArgPair>::const_iterator const_param_arg_iterator;

  param_arg_iterator param_arg_begin() { return ParamArgs.begin(); }
  param_arg_iterator param_arg_end() { return ParamArgs.end(); }
  const_param_arg_iterator param_arg_begin() const { return ParamArgs.begin(); }
  const_param_arg_iterator param_arg_end() const { return ParamArgs.end(); }
  unsigned param_arg_size() const { return ParamArgs.size(); }

  typedef SmallVectorImpl<ArgPair>::iterator arg_iterator;
  typedef SmallVectorImpl<ArgPair>::const_iterator const_arg_iterator;

  arg_iterator arg_begin() { return Args.begin(); }
  arg_iterator arg_end() { return Args.end(); }
  const_arg_iterator arg_begin() const { return Args.begin(); }
  const_arg_iterator arg_end() const { return Args.end(); }
  unsigned arg_size() const { return Args.size(); }

  void addParamArgument(VerilogParameter *param, VerilogParamConst value);
  void addParamArgument(StringRef paramName, VerilogParamConst value);
  void addArgument(VerilogSignal *arg, VerilogExpr *value);
  void addArgument(StringRef argName, VerilogExpr *value);

  void print(raw_ostream &OS) const;
};

// Events

enum VerilogEventKind {
  EvK_Change,
  EvK_Posedge,
  EvK_Negedge
};

class VerilogEvent {
  VerilogEventKind Kind;
  VerilogSignalRef *Signal;

public:
  VerilogEvent(VerilogEventKind kind, VerilogSignalRef *signal)
    : Kind(kind), Signal(signal) {}

  VerilogEventKind getKind() const { return Kind; }
  VerilogSignalRef *getSignal() const { return Signal; }

  void print(raw_ostream &OS) const;
};

// Statements

class VerilogStmt {
public:
  enum VerilogStmtClass {
    VS_AlwaysClass = 1,
    VS_AssignClass,
    VS_CompoundClass,
    VS_IfClass,
    VS_InitialClass,
    VS_SwitchClass,

    VS_ConstClass,
    VS_ParamRefClass,
    VS_DefineRefClass,
    VS_SignalRefClass,
    VS_PartSelectClass,
    VS_ConcatClass,
    VS_ReplicationClass,
    VS_UnaryOperatorClass,
    VS_BinaryOperatorClass,
    VS_ConditionalOperatorClass,

    firstVerilogExprConstant = VS_ConstClass,
    lastVerilogExprConstant = VS_ConditionalOperatorClass
  };

private:
  VerilogStmtClass Class;

protected:
  explicit VerilogStmt(VerilogStmtClass C)
    : Class(C) {}

public:
  VerilogStmtClass getClass() const { return Class; }

  void print(raw_ostream &OS) const;
};

class VerilogAlways : public VerilogStmt {
  SmallVector<VerilogEvent, 4> Events;
  VerilogCompound *Body;

public:
  explicit VerilogAlways(VerilogCompound *body)
    : VerilogStmt(VS_AlwaysClass), Events(), Body(body) {}
  VerilogAlways(VerilogEvent event, VerilogCompound *body)
    : VerilogStmt(VS_AlwaysClass), Events(1, event), Body(body) {}

  typedef SmallVectorImpl<VerilogEvent>::iterator event_iterator;
  typedef SmallVectorImpl<VerilogEvent>::const_iterator const_event_iterator;

  event_iterator event_begin() { return Events.begin(); }
  event_iterator event_end() { return Events.end(); }
  const_event_iterator event_begin() const { return Events.begin(); }
  const_event_iterator event_end() const { return Events.end(); }
  unsigned event_size() const { return Events.size(); }

  VerilogCompound *getBody() const { return Body; }

  void addEvent(VerilogEvent event) {
    Events.push_back(event);
  }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_AlwaysClass;
  }
};

class VerilogAssign : public VerilogStmt {
  VerilogSignalRef *LHS;
  VerilogExpr *RHS;
  bool Blocking;
  bool TopLevel;

public:
  VerilogAssign(VerilogSignalRef *lhs, VerilogExpr *rhs, bool blocking = false)
    : VerilogStmt(VS_AssignClass), LHS(lhs), RHS(rhs), Blocking(blocking),
      TopLevel(false) {}

  VerilogSignalRef *getLHS() const { return LHS; }
  VerilogExpr *getRHS() const { return RHS; }
  bool isBlocking() const { return Blocking; }
  bool isTopLevel() const { return TopLevel; }
  void setAsTopLevel() { TopLevel = true; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_AssignClass;
  }
};

class VerilogCompound : public VerilogStmt {
  SmallVector<VerilogStmt*, 32> Body;

public:
  VerilogCompound()
    : VerilogStmt(VS_CompoundClass) {}

  typedef SmallVectorImpl<VerilogStmt*>::iterator iterator;
  typedef SmallVectorImpl<VerilogStmt*>::const_iterator const_iterator;

  iterator begin() { return Body.begin(); }
  iterator end() { return Body.end(); }
  const_iterator begin() const { return Body.begin(); }
  const_iterator end() const { return Body.end(); }

  VerilogCompound *addStmt(VerilogStmt *S) {
    Body.push_back(S);
    return this;
  }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_CompoundClass;
  }
};

class VerilogIf : public VerilogStmt {
  VerilogExpr *Cond;
  VerilogStmt *Then;
  VerilogStmt *Else;

public:
  VerilogIf(VerilogExpr *cond, VerilogStmt *thenS)
    : VerilogStmt(VS_IfClass), Cond(cond), Then(thenS), Else(NULL) {}
  VerilogIf(VerilogExpr *cond, VerilogStmt *thenS, VerilogStmt *elseS)
    : VerilogStmt(VS_IfClass), Cond(cond), Then(thenS), Else(elseS) {}

  VerilogExpr *getCond() const { return Cond; }
  VerilogStmt *getThen() const { return Then; }
  VerilogStmt *getElse() const { return Else; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_IfClass;
  }
};

class VerilogInitial : public VerilogStmt {
  VerilogCompound *Body;

public:
  explicit VerilogInitial(VerilogCompound *body)
    : VerilogStmt(VS_InitialClass), Body(body) {}

  VerilogCompound *getBody() const { return Body; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_InitialClass;
  }
};

class VerilogSwitch : public VerilogStmt {
  VerilogExpr *Cond;
  SmallVector<VerilogExpr*, 32> Cases;
  SmallVector<VerilogStmt*, 32> CaseStmts;
  VerilogStmt *DefaultStmt;

public:
  explicit VerilogSwitch(VerilogExpr *cond)
    : VerilogStmt(VS_SwitchClass), Cond(cond), DefaultStmt(NULL) {}

  VerilogExpr *getCond() const { return Cond; }
  unsigned getNumCases() const { return Cases.size(); }
  VerilogExpr *getCase(unsigned Index) const {
    assert(Index < Cases.size());
    return Cases[Index];
  }
  VerilogStmt *getCaseStmt(unsigned Index) const {
    assert(Index < CaseStmts.size());
    return CaseStmts[Index];
  }
  bool hasDefaultStmt() const { return DefaultStmt != NULL; }
  VerilogStmt *getDefaultStmt() const { return DefaultStmt; }

  VerilogSwitch *addCase(VerilogExpr *value, VerilogStmt *S) {
    Cases.push_back(value);
    CaseStmts.push_back(S);
    return this;
  }
  VerilogSwitch *addDefault(VerilogStmt *S) {
    DefaultStmt = S;
    return this;
  }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_SwitchClass;
  }
};

class VerilogExpr : public VerilogStmt {
protected:
  explicit VerilogExpr(VerilogStmtClass C)
    : VerilogStmt(C) {}

public:
  static bool classof(const VerilogStmt *T) {
    return (T->getClass() >= firstVerilogExprConstant &&
            T->getClass() <= lastVerilogExprConstant);
  }
};

enum VerilogValueRadixKind {
  VR_Binary,
  VR_Octal,
  VR_Decimal,
  VR_Hexa
};

class VerilogConst : public VerilogExpr {
  VerilogValueRadixKind RadixKind;
  std::string Value;
  unsigned Size;

public:
  VerilogConst(VerilogValueRadixKind radix, StringRef value, unsigned size = 0);
  VerilogConst(VerilogValueRadixKind radix, uint64_t value, unsigned size = 0);

  VerilogValueRadixKind getRadixKind() const { return RadixKind; }
  char getRadixCode() const;
  StringRef getValue() const { return Value; }
  bool isSized() const { return Size > 0; }
  unsigned getSize() const { return Size; }

  std::string getAsString() const;

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_ConstClass;
  }
};

class VerilogParamRef : public VerilogExpr {
  VerilogParameter *Param;

public:
  explicit VerilogParamRef(VerilogParameter *param)
    : VerilogExpr(VS_ParamRefClass), Param(param) {}

  VerilogParameter *getParameter() const { return Param; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_ParamRefClass;
  }
};

class VerilogDefineRef : public VerilogExpr {
  VerilogDefine *Define;

public:
  explicit VerilogDefineRef(VerilogDefine *define)
    : VerilogExpr(VS_DefineRefClass), Define(define) {}

  VerilogDefine *getDefine() const { return Define; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_DefineRefClass;
  }
};

class VerilogSignalRef : public VerilogExpr {
  VerilogSignal *Signal;
  VerilogExpr *ArrayIndex;
  VerilogParamConst MSB, LSB;

public:
  explicit VerilogSignalRef(VerilogSignal *signal);
  VerilogSignalRef(VerilogSignal *signal, VerilogParamConst msb,
                   VerilogParamConst lsb);
  VerilogSignalRef(VerilogSignal *signal, VerilogExpr *index);
  VerilogSignalRef(VerilogSignal *signal, VerilogExpr *index,
                   VerilogParamConst msb, VerilogParamConst lsb);

  VerilogSignal *getSignal() const { return Signal; }
  bool hasArrayIndex() const { return ArrayIndex != NULL; }
  VerilogExpr *getArrayIndex() const { return ArrayIndex; }
  bool hasVectorRange() const;
  VerilogParamConst getMSB() const { return MSB; }
  VerilogParamConst getLSB() const { return LSB; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_SignalRefClass;
  }
};

class VerilogPartSelect : public VerilogExpr {
  VerilogSignal *Signal;
  VerilogExpr *Offset;
  VerilogParamConst Width;

public:
  VerilogPartSelect(VerilogSignal *signal, VerilogExpr *offset,
                    VerilogParamConst width);

  VerilogSignal *getSignal() const { return Signal; }
  VerilogExpr *getOffset() const { return Offset; }
  VerilogParamConst getWidth() const { return Width; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_PartSelectClass;
  }
};

class VerilogConcat : public VerilogExpr {
  SmallVector<VerilogExpr*, 4> Operands;

public:
  VerilogConcat(VerilogExpr *A, VerilogExpr *B)
    : VerilogExpr(VS_ConcatClass) {
    Operands.push_back(A);
    Operands.push_back(B);
  }
  VerilogConcat(VerilogExpr *A, VerilogExpr *B, VerilogExpr *C)
    : VerilogExpr(VS_ConcatClass) {
    Operands.push_back(A);
    Operands.push_back(B);
    Operands.push_back(C);
  }
  VerilogConcat(ArrayRef<VerilogExpr*> operands)
    : VerilogExpr(VS_ConcatClass), Operands(operands.begin(), operands.end()) {
  }

  unsigned getNumOperands() const { return Operands.size(); }
  VerilogExpr *getOperand(unsigned Index) const {
    assert(Index < Operands.size() && "index out of range");
    return Operands[Index];
  }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_ConcatClass;
  }
};

class VerilogReplication : public VerilogExpr {
  unsigned Number;
  VerilogExpr *Operand;

public:
  VerilogReplication(unsigned number, VerilogExpr *operand)
    : VerilogExpr(VS_ReplicationClass), Number(number), Operand(operand) {}

  unsigned getNumber() const { return Number; }
  VerilogExpr *getOperand() const { return Operand; }

  static bool classof(class VerilogStmt *T) {
    return T->getClass() == VS_ReplicationClass;
  }
};

enum VerilogUnaryOpKind {
  VUO_PLUS, VUO_MINUS, // arithmetic
  VUO_LNOT, // logical not
  VUO_NOT, // bit-wise
  VUO_RAND, VUO_RNAND, VUO_ROR, VUO_RNOR, VUO_RXOR, VUO_RXNOR // reduction
};

class VerilogUnaryOperator : public VerilogExpr {
  VerilogUnaryOpKind Op;
  VerilogExpr *Operand;

public:
  VerilogUnaryOperator(VerilogUnaryOpKind op, VerilogExpr *operand)
    : VerilogExpr(VS_UnaryOperatorClass), Op(op), Operand(operand) {}

  VerilogUnaryOpKind getOpcode() const { return Op; }
  VerilogExpr *getOperand() const { return Operand; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_UnaryOperatorClass;
  }
};

enum VerilogBinaryOpKind {
  VBO_ADD, VBO_SUB, VBO_MUL, VBO_DIV, VBO_MOD, // arithmetic and modulus
  VBO_GT, VBO_GE, VBO_LT, VBO_LE, // relational
  VBO_LAND, VBO_LOR, // logical and/or
  VBO_LEQ, VBO_LNE, // logical equality
  VBO_CEQ, VBO_CNE, // case equality
  VBO_AND, VBO_OR, VBO_XOR, VBO_XNOR, // bit-wise
  VBO_LSHL, VBO_LSHR, // logical shift
  VBO_ASHL, VBO_ASHR // arithmetic shift
};

class VerilogBinaryOperator : public VerilogExpr {
  VerilogBinaryOpKind Op;
  VerilogExpr *LHS;
  VerilogExpr *RHS;

public:
  VerilogBinaryOperator(VerilogBinaryOpKind op, VerilogExpr *lhs,
                        VerilogExpr *rhs)
    : VerilogExpr(VS_BinaryOperatorClass), Op(op), LHS(lhs), RHS(rhs) {}

  VerilogBinaryOpKind getOpcode() const { return Op; }
  VerilogExpr *getLHS() const { return LHS; }
  VerilogExpr *getRHS() const { return RHS; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_BinaryOperatorClass;
  }
};

class VerilogConditionalOperator : public VerilogExpr {
  VerilogExpr *Cond;
  VerilogExpr *LHS;
  VerilogExpr *RHS;

public:
  VerilogConditionalOperator(VerilogExpr *cond, VerilogExpr *lhs,
                             VerilogExpr *rhs)
    : VerilogExpr(VS_ConditionalOperatorClass), Cond(cond), LHS(lhs),
      RHS(rhs) {}

  VerilogExpr *getCond() const { return Cond; }
  VerilogExpr *getLHS() const { return LHS; }
  VerilogExpr *getRHS() const { return RHS; }

  static bool classof(const VerilogStmt *T) {
    return T->getClass() == VS_ConditionalOperatorClass;
  }
};

// Helper classes for supporting implicit cast

class VerilogParamConstOrInt : public VerilogParamConst {
public:
  VerilogParamConstOrInt() : VerilogParamConst() {}
  VerilogParamConstOrInt(const VerilogParamConst &C) : VerilogParamConst(C) {}
  VerilogParamConstOrInt(int value) : VerilogParamConst(value) {}
};

class VerilogExprPtrOrSignalPtr {
  VerilogExpr *E;
public:
  VerilogExprPtrOrSignalPtr(VerilogExpr *e) : E(e) {}
  VerilogExprPtrOrSignalPtr(VerilogSignal *s) : E(new VerilogSignalRef(s)) {}
  operator VerilogExpr*() { return E; }
  VerilogExpr &operator*() { return *E; }
  VerilogExpr *operator->() { return E; }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_VERILOG_H
