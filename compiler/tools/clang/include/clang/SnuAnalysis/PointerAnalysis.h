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

#ifndef LLVM_CLANG_SNU_ANALYSIS_POINTERANALYSIS_H
#define LLVM_CLANG_SNU_ANALYSIS_POINTERANALYSIS_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAnalysis/MemoryAccess.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "llvm/ADT/DenseSet.h"

namespace clang {

namespace snu {

class IndexedVarDecl;
class WCFG;
class WExpr;
class WParmVarDecl;
class WVarDecl;

class AliasSet {
  WCFG *Program;

  typedef OrderedDenseMap<const WVarDecl*, unsigned> VarAliasMapTy;
  typedef OrderedDenseMap<const WExpr*, unsigned> ExprAliasMapTy;

  unsigned MaxAlias;
  VarAliasMapTy VarAlias;
  ExprAliasMapTy ExprAlias;
  llvm::DenseSet<unsigned> PointsToParameter;

public:
  explicit AliasSet(WCFG *program);
  AliasSet(WCFG *program, MemoryAccessTrace::FilterTy Filter);

  WCFG *getProgram() const { return Program; }

  typedef VarAliasMapTy::const_iterator var_iterator;
  typedef ExprAliasMapTy::const_iterator expr_iterator;
  var_iterator var_begin() const { return VarAlias.begin(); }
  var_iterator var_end() const { return VarAlias.end(); }
  expr_iterator expr_begin() const { return ExprAlias.begin(); }
  expr_iterator expr_end() const { return ExprAlias.end(); }

  bool isUsed(const WVarDecl *V) const;
  unsigned getAlias(const WVarDecl *V) const;
  bool isMemoryAccess(const WExpr *E) const;
  unsigned getAlias(const WExpr *E) const;

  bool isPointsToParameter(unsigned Alias) const;
  bool isPointsToParameter(const WVarDecl *V) const;
  bool isPointsToParameter(const WExpr *E) const;

  void setAlias(const WVarDecl* V, unsigned A);
  void setAlias(const WExpr *E, unsigned A);

  MemoryAccessRelation *CreateRelation(const MemoryAccessTrace &Trace) const;

  void print(raw_ostream &OS, const ASTContext &ASTCtx) const;
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_POINTERANALYSIS_H
