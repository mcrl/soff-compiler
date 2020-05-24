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

#include "clang/SnuSynthesis/PlatformContext.h"
#include "CGCommon.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Attr.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"
#include <fstream>
#include <map>
#include <string>

namespace clang {

namespace snu {

// PlatformVariables

PlatformVariables::PlatformVariables(StringRef SetupFile) {
  LoadTargetVars(SetupFile);
}

unsigned PlatformVariables::GetVar(StringRef key) const {
  std::map<std::string, unsigned>::const_iterator it = Var.find(key);
  if (it == Var.end()) {
    llvm::errs() << "error: the variable " << key << " is not found.\n";
  }
  assert(it != Var.end());
  return it->second;
}

unsigned PlatformVariables::GetVar(StringRef group, StringRef sub) const {
  std::string key;
  key += group;
  key += '.';
  key += sub;
  return GetVar(key);
}

bool PlatformVariables::LoadTargetVars(StringRef TargetFile) {
  std::ifstream f(TargetFile.data());
  if (!f.is_open()) {
    llvm::errs() << "error: " << TargetFile << " does not exist\n";
    return false;
  }

  while (!f.eof()) {
    std::string key;
    f >> key;
    if (f.fail()) break;

    if (key == "#include") {
      std::string filename;
      f >> filename;
      if (f.fail()) {
        llvm::errs() << TargetFile << ": error: no filename in #include\n";
        break;
      }
      if (!LoadTargetVars(filename)) {
        break;
      }

    } else {
      unsigned value;
      f >> value;
      if (f.fail()) {
        llvm::errs() << TargetFile << ": error: no value for \'" << key
                     << "\'\n";
        break;
      }
      assert(Var.count(key) == 0);
      Var[key] = value;
    }
  }
  return true;
}

// PlatformIPCore / PlatformIPCollection

PlatformIPCore::PlatformIPCore(VerilogModule *module)
  : Module(module), Lmax((unsigned)-1), Lkind(LK_Undefined) {
}

PlatformIPCore::PlatformIPCore(VerilogModule *module, unsigned lmax,
                               PlatformIPCore::LatencyKind lkind)
  : Module(module), Lmax(lmax), Lkind(lkind) {
  assert(Lkind != LK_Undefined);
}

unsigned PlatformIPCore::getLmax() const {
  assert(Lkind != LK_Undefined);
  return Lmax;
}

unsigned PlatformIPCore::getNstall() const {
  // Some fixed-latency IP cores from FPGA vendors receive only one "enable"
  // signal shared by all pipepline stages and are possible to stall while
  // holding only two work-items. Such cores have Nstall of two, not Lmax + 1.
  // --[  bubble     bubble     bubble   work-item1](          )-- =>
  // --[work-item2   bubble     bubble     bubble  ](work-item1)--
  assert(Lkind != LK_Undefined);
  return (Lkind == LK_FixedSingle ? 2 : Lmax + 1);
}

PlatformIPCore::LatencyKind PlatformIPCore::InterpretAsLkind(unsigned v) {
  switch (v) {
    case 0: return PlatformIPCore::LK_Fixed;
    case 1: return PlatformIPCore::LK_Variable;
    case 2: return PlatformIPCore::LK_FixedSingle;
    default: llvm_unreachable("invalid type");
  }
}

PlatformIPCollection::PlatformIPCollection(const ASTContext &C,
                                           const PlatformVariables &V)
  : ASTCtx(C), PVars(V) {
  // Unary operators
  AddUnaryOpIP("minus_i32",    32, 32);
  AddUnaryOpIP("minus_i64",    64, 64);
  AddUnaryOpIP("minus_float",  32, 32);
  AddUnaryOpIP("minus_double", 64, 64);
  AddUnaryOpIP("lnot_i8",       8, 32);
  AddUnaryOpIP("lnot_u8",       8, 32);
  AddUnaryOpIP("lnot_i16",     16, 32);
  AddUnaryOpIP("lnot_u16",     16, 32);
  AddUnaryOpIP("lnot_i32",     32, 32);
  AddUnaryOpIP("lnot_u32",     32, 32);
  AddUnaryOpIP("lnot_i64",     64, 32);
  AddUnaryOpIP("lnot_u64",     64, 32);
  AddUnaryOpIP("lnot_v_i32",   32, 32);
  AddUnaryOpIP("lnot_v_u32",   32, 32);
  AddUnaryOpIP("lnot_v_i64",   64, 32);
  AddUnaryOpIP("lnot_v_u64",   64, 32);
  AddUnaryOpIP("itof_i32",     32, 32);
  AddUnaryOpIP("itof_u32",     32, 32);
  AddUnaryOpIP("itof_i64",     64, 32);
  AddUnaryOpIP("itof_u64",     64, 32);
  AddUnaryOpIP("itod_i32",     32, 64);
  AddUnaryOpIP("itod_u32",     32, 64);
  AddUnaryOpIP("itod_i64",     64, 64);
  AddUnaryOpIP("itod_u64",     64, 64);
  AddUnaryOpIP("ftoi_i32",     32, 32);
  AddUnaryOpIP("ftoi_u32",     32, 32);
  AddUnaryOpIP("ftoi_i64",     32, 64);
  AddUnaryOpIP("ftoi_u64",     32, 64);
  AddUnaryOpIP("dtoi_i32",     64, 32);
  AddUnaryOpIP("dtoi_u32",     64, 32);
  AddUnaryOpIP("dtoi_i64",     64, 64);
  AddUnaryOpIP("dtoi_u64",     64, 64);
  AddUnaryOpIP("ftod",         32, 64);
  AddUnaryOpIP("dtof",         64, 32);
  AddUnaryOpIP("cos_float",    32, 32);
  AddUnaryOpIP("cos_double",   64, 64);
  AddUnaryOpIP("sin_float",    32, 32);
  AddUnaryOpIP("sin_double",   64, 64);
  AddUnaryOpIP("tan_float",    32, 32);
  AddUnaryOpIP("tan_double",   64, 64);
  AddUnaryOpIP("acos_float",   32, 32);
  AddUnaryOpIP("acos_double",  64, 64);
  AddUnaryOpIP("asin_float",   32, 32);
  AddUnaryOpIP("asin_double",  64, 64);
  AddUnaryOpIP("atan_float",   32, 32);
  AddUnaryOpIP("atan_double",  64, 64);
  AddUnaryOpIP("exp_float",    32, 32);
  AddUnaryOpIP("exp_double",   64, 64);
  AddUnaryOpIP("log_float",    32, 32);
  AddUnaryOpIP("log_double",   64, 64);
  AddUnaryOpIP("log2_float",   32, 32);
  AddUnaryOpIP("log2_double",  64, 64);
  AddUnaryOpIP("log10_float",  32, 32);
  AddUnaryOpIP("log10_double", 64, 64);
  AddUnaryOpIP("sqrt_float",   32, 32);
  AddUnaryOpIP("sqrt_double",  64, 64);
  AddUnaryOpIP("rsqrt_float",  32, 32);
  AddUnaryOpIP("rsqrt_double", 64, 64);
  AddUnaryOpIP("ceil_float",   32, 32);
  AddUnaryOpIP("ceil_double",  64, 64);
  AddUnaryOpIP("floor_float",  32, 32);
  AddUnaryOpIP("floor_double", 64, 64);
  AddUnaryOpIP("round_float",  32, 32);
  AddUnaryOpIP("round_double", 64, 64);
  AddUnaryOpIP("trunc_float",  32, 32);
  AddUnaryOpIP("trunc_double", 64, 64);
  AddUnaryOpIP("abs_i8",        8,  8);
  AddUnaryOpIP("abs_i16",      16, 16);
  AddUnaryOpIP("abs_i32",      32, 32);
  AddUnaryOpIP("abs_i64",      64, 64);
  AddUnaryOpIP("abs_float",    32, 32);
  AddUnaryOpIP("abs_double",   64, 64);

  // Binary operators
  AddBinaryOpIP("mul_i8",         8, 8, 8);
  AddBinaryOpIP("mul_u8",         8, 8, 8);
  AddBinaryOpIP("mul_i16",         16, 16, 16);
  AddBinaryOpIP("mul_u16",         16, 16, 16);
  AddBinaryOpIP("mul_i32",         32, 32, 32);
  AddBinaryOpIP("mul_u32",         32, 32, 32);
  AddBinaryOpIP("mul_i64",         64, 64, 64);
  AddBinaryOpIP("mul_u64",         64, 64, 64);
  AddBinaryOpIP("mul_float",       32, 32, 32);
  AddBinaryOpIP("mul_double",      64, 64, 64);
  AddBinaryOpIP("div_i32",         32, 32, 32);
  AddBinaryOpIP("div_u32",         32, 32, 32);
  AddBinaryOpIP("div_i64",         64, 64, 64);
  AddBinaryOpIP("div_u64",         64, 64, 64);
  AddBinaryOpIP("div_float",       32, 32, 32);
  AddBinaryOpIP("div_double",      64, 64, 64);
  AddBinaryOpIP("rem_i32",         32, 32, 32);
  AddBinaryOpIP("rem_u32",         32, 32, 32);
  AddBinaryOpIP("rem_i64",         64, 64, 64);
  AddBinaryOpIP("rem_u64",         64, 64, 64);
  AddBinaryOpIP("add_i32",         32, 32, 32);
  AddBinaryOpIP("add_u32",         32, 32, 32);
  AddBinaryOpIP("add_i64",         64, 64, 64);
  AddBinaryOpIP("add_u64",         64, 64, 64);
  AddBinaryOpIP("add_float",       32, 32, 32);
  AddBinaryOpIP("add_double",      64, 64, 64);
  AddBinaryOpIP("sub_i32",         32, 32, 32);
  AddBinaryOpIP("sub_u32",         32, 32, 32);
  AddBinaryOpIP("sub_i64",         64, 64, 64);
  AddBinaryOpIP("sub_u64",         64, 64, 64);
  AddBinaryOpIP("sub_float",       32, 32, 32);
  AddBinaryOpIP("sub_double",      64, 64, 64);
  AddBinaryOpIP("shl_i32",         32, 32, 32);
  AddBinaryOpIP("shl_u32",         32, 32, 32);
  AddBinaryOpIP("shl_i64",         64, 32, 64);
  AddBinaryOpIP("shl_u64",         64, 32, 64);
  AddBinaryOpIP("shr_i32",         32, 32, 32);
  AddBinaryOpIP("shr_u32",         32, 32, 32);
  AddBinaryOpIP("shr_i64",         64, 32, 64);
  AddBinaryOpIP("shr_u64",         64, 32, 64);
  AddBinaryOpIP("cmp_lt_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_lt_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_lt_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_lt_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_lt_float",    32, 32, 32);
  AddBinaryOpIP("cmp_lt_double",   64, 64, 32);
  AddBinaryOpIP("cmp_gt_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_gt_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_gt_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_gt_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_gt_float",    32, 32, 32);
  AddBinaryOpIP("cmp_gt_double",   64, 64, 32);
  AddBinaryOpIP("cmp_le_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_le_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_le_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_le_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_le_float",    32, 32, 32);
  AddBinaryOpIP("cmp_le_double",   64, 64, 32);
  AddBinaryOpIP("cmp_ge_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_ge_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_ge_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_ge_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_ge_float",    32, 32, 32);
  AddBinaryOpIP("cmp_ge_double",   64, 64, 32);
  AddBinaryOpIP("cmp_lt_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_lt_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_lt_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_lt_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_lt_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_lt_v_double", 64, 64, 32);
  AddBinaryOpIP("cmp_gt_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_gt_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_gt_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_gt_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_gt_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_gt_v_double", 64, 64, 32);
  AddBinaryOpIP("cmp_le_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_le_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_le_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_le_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_le_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_le_v_double", 64, 64, 32);
  AddBinaryOpIP("cmp_ge_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_ge_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_ge_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_ge_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_ge_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_ge_v_double", 64, 64, 32);
  AddBinaryOpIP("cmp_eq_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_eq_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_eq_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_eq_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_eq_float",    32, 32, 32);
  AddBinaryOpIP("cmp_eq_double",   64, 64, 32);
  AddBinaryOpIP("cmp_ne_i32",      32, 32, 32);
  AddBinaryOpIP("cmp_ne_u32",      32, 32, 32);
  AddBinaryOpIP("cmp_ne_i64",      64, 64, 32);
  AddBinaryOpIP("cmp_ne_u64",      64, 64, 32);
  AddBinaryOpIP("cmp_ne_float",    32, 32, 32);
  AddBinaryOpIP("cmp_ne_double",   64, 64, 32);
  AddBinaryOpIP("cmp_eq_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_eq_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_eq_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_eq_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_eq_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_eq_v_double", 64, 64, 32);
  AddBinaryOpIP("cmp_ne_v_i32",    32, 32, 32);
  AddBinaryOpIP("cmp_ne_v_u32",    32, 32, 32);
  AddBinaryOpIP("cmp_ne_v_i64",    64, 64, 32);
  AddBinaryOpIP("cmp_ne_v_u64",    64, 64, 32);
  AddBinaryOpIP("cmp_ne_v_float",  32, 32, 32);
  AddBinaryOpIP("cmp_ne_v_double", 64, 64, 32);
  AddBinaryOpIP("land_u8",         8,  8,  32);
  AddBinaryOpIP("land_i32",        32, 32, 32);
  AddBinaryOpIP("land_u32",        32, 32, 32);
  AddBinaryOpIP("land_i64",        64, 64, 32);
  AddBinaryOpIP("land_u64",        64, 64, 32);
  AddBinaryOpIP("land_v_i32",      32, 32, 32);
  AddBinaryOpIP("land_v_u32",      32, 32, 32);
  AddBinaryOpIP("land_v_i64",      64, 64, 32);
  AddBinaryOpIP("land_v_u64",      64, 64, 32);
  AddBinaryOpIP("lor_u8",          8,  8,  32);
  AddBinaryOpIP("lor_i32",         32, 32, 32);
  AddBinaryOpIP("lor_u32",         32, 32, 32);
  AddBinaryOpIP("lor_i64",         64, 64, 32);
  AddBinaryOpIP("lor_u64",         64, 64, 32);
  AddBinaryOpIP("lor_v_i32",       32, 32, 32);
  AddBinaryOpIP("lor_v_u32",       32, 32, 32);
  AddBinaryOpIP("lor_v_i64",       64, 64, 32);
  AddBinaryOpIP("lor_v_u64",       64, 64, 32);
  AddBinaryOpIP("pow_float",       32, 32, 32);
  AddBinaryOpIP("pow_double",      64, 64, 64);
  AddBinaryOpIP("max_i8",           8,  8,  8);
  AddBinaryOpIP("max_u8",           8,  8,  8);
  AddBinaryOpIP("max_i16",         16, 16, 16);
  AddBinaryOpIP("max_u16",         16, 16, 16);
  AddBinaryOpIP("max_i32",         32, 32, 32);
  AddBinaryOpIP("max_u32",         32, 32, 32);
  AddBinaryOpIP("max_i64",         64, 64, 64);
  AddBinaryOpIP("max_u64",         64, 64, 64);
  AddBinaryOpIP("min_i8",           8,  8,  8);
  AddBinaryOpIP("min_u8",           8,  8,  8);
  AddBinaryOpIP("min_i16",         16, 16, 16);
  AddBinaryOpIP("min_u16",         16, 16, 16);
  AddBinaryOpIP("min_i32",         32, 32, 32);
  AddBinaryOpIP("min_u32",         32, 32, 32);
  AddBinaryOpIP("min_i64",         64, 64, 64);
  AddBinaryOpIP("min_u64",         64, 64, 64);
  AddBinaryOpIP("mul24_i32",       32, 32, 32);
  AddBinaryOpIP("mul24_u32",       32, 32, 32);

  AddNullaryAtomicIP("atomic_inc_global_i32", 32);
  AddNullaryAtomicIP("atomic_inc_global_u32", 32);
  AddNullaryAtomicIP("atomic_inc_global_i64", 64);
  AddNullaryAtomicIP("atomic_inc_global_u64", 64);
  AddNullaryAtomicIP("atomic_dec_global_i32", 32);
  AddNullaryAtomicIP("atomic_dec_global_u32", 32);
  AddNullaryAtomicIP("atomic_dec_global_i64", 64);
  AddNullaryAtomicIP("atomic_dec_global_u64", 64);
  AddNullaryAtomicIP("atomic_inc_local_i32", 32);
  AddNullaryAtomicIP("atomic_inc_local_u32", 32);
  AddNullaryAtomicIP("atomic_inc_local_i64", 64);
  AddNullaryAtomicIP("atomic_inc_local_u64", 64);
  AddNullaryAtomicIP("atomic_dec_local_i32", 32);
  AddNullaryAtomicIP("atomic_dec_local_u32", 32);
  AddNullaryAtomicIP("atomic_dec_local_i64", 64);
  AddNullaryAtomicIP("atomic_dec_local_u64", 64);

  AddUnaryAtomicIP("atomic_add_global_i32",  32);
  AddUnaryAtomicIP("atomic_add_global_u32",  32);
  AddUnaryAtomicIP("atomic_add_global_i64",  64);
  AddUnaryAtomicIP("atomic_add_global_u64",  64);
  AddUnaryAtomicIP("atomic_sub_global_i32",  32);
  AddUnaryAtomicIP("atomic_sub_global_u32",  32);
  AddUnaryAtomicIP("atomic_sub_global_i64",  64);
  AddUnaryAtomicIP("atomic_sub_global_u64",  64);
  AddUnaryAtomicIP("atomic_xchg_global_i32", 32);
  AddUnaryAtomicIP("atomic_xchg_global_u32", 32);
  AddUnaryAtomicIP("atomic_xchg_global_i64", 64);
  AddUnaryAtomicIP("atomic_xchg_global_u64", 64);
  AddUnaryAtomicIP("atomic_min_global_i32",  32);
  AddUnaryAtomicIP("atomic_min_global_u32",  32);
  AddUnaryAtomicIP("atomic_min_global_i64",  64);
  AddUnaryAtomicIP("atomic_min_global_u64",  64);
  AddUnaryAtomicIP("atomic_max_global_i32",  32);
  AddUnaryAtomicIP("atomic_max_global_u32",  32);
  AddUnaryAtomicIP("atomic_max_global_i64",  64);
  AddUnaryAtomicIP("atomic_max_global_u64",  64);
  AddUnaryAtomicIP("atomic_and_global_i32",  32);
  AddUnaryAtomicIP("atomic_and_global_u32",  32);
  AddUnaryAtomicIP("atomic_and_global_i64",  64);
  AddUnaryAtomicIP("atomic_and_global_u64",  64);
  AddUnaryAtomicIP("atomic_or_global_i32",   32);
  AddUnaryAtomicIP("atomic_or_global_u32",   32);
  AddUnaryAtomicIP("atomic_or_global_i64",   64);
  AddUnaryAtomicIP("atomic_or_global_u64",   64);
  AddUnaryAtomicIP("atomic_xor_global_i32",  32);
  AddUnaryAtomicIP("atomic_xor_global_u32",  32);
  AddUnaryAtomicIP("atomic_xor_global_i64",  64);
  AddUnaryAtomicIP("atomic_xor_global_u64",  64);
  AddUnaryAtomicIP("atomic_add_local_i32",  32);
  AddUnaryAtomicIP("atomic_add_local_u32",  32);
  AddUnaryAtomicIP("atomic_add_local_i64",  64);
  AddUnaryAtomicIP("atomic_add_local_u64",  64);
  AddUnaryAtomicIP("atomic_sub_local_i32",  32);
  AddUnaryAtomicIP("atomic_sub_local_u32",  32);
  AddUnaryAtomicIP("atomic_sub_local_i64",  64);
  AddUnaryAtomicIP("atomic_sub_local_u64",  64);
  AddUnaryAtomicIP("atomic_xchg_local_i32", 32);
  AddUnaryAtomicIP("atomic_xchg_local_u32", 32);
  AddUnaryAtomicIP("atomic_xchg_local_i64", 64);
  AddUnaryAtomicIP("atomic_xchg_local_u64", 64);
  AddUnaryAtomicIP("atomic_min_local_i32",  32);
  AddUnaryAtomicIP("atomic_min_local_u32",  32);
  AddUnaryAtomicIP("atomic_min_local_i64",  64);
  AddUnaryAtomicIP("atomic_min_local_u64",  64);
  AddUnaryAtomicIP("atomic_max_local_i32",  32);
  AddUnaryAtomicIP("atomic_max_local_u32",  32);
  AddUnaryAtomicIP("atomic_max_local_i64",  64);
  AddUnaryAtomicIP("atomic_max_local_u64",  64);
  AddUnaryAtomicIP("atomic_and_local_i32",  32);
  AddUnaryAtomicIP("atomic_and_local_u32",  32);
  AddUnaryAtomicIP("atomic_and_local_i64",  64);
  AddUnaryAtomicIP("atomic_and_local_u64",  64);
  AddUnaryAtomicIP("atomic_or_local_i32",   32);
  AddUnaryAtomicIP("atomic_or_local_u32",   32);
  AddUnaryAtomicIP("atomic_or_local_i64",   64);
  AddUnaryAtomicIP("atomic_or_local_u64",   64);
  AddUnaryAtomicIP("atomic_xor_local_i32",  32);
  AddUnaryAtomicIP("atomic_xor_local_u32",  32);
  AddUnaryAtomicIP("atomic_xor_local_i64",  64);
  AddUnaryAtomicIP("atomic_xor_local_u64",  64);

  AddBinaryAtomicIP("atomic_cmpxchg_global_i32", 32);
  AddBinaryAtomicIP("atomic_cmpxchg_global_u32", 32);
  AddBinaryAtomicIP("atomic_cmpxchg_global_i64", 64);
  AddBinaryAtomicIP("atomic_cmpxchg_global_u64", 64);
  AddBinaryAtomicIP("atomic_cmpxchg_local_i32", 32);
  AddBinaryAtomicIP("atomic_cmpxchg_local_u32", 32);
  AddBinaryAtomicIP("atomic_cmpxchg_local_i64", 64);
  AddBinaryAtomicIP("atomic_cmpxchg_local_u64", 64);

#define _MODULE(X) \
    { VerilogModule *Module = new VerilogModule(#X);
#define _PARAM(Var, X, DefaultValue) \
    VerilogParameter *Var = Module->CreateParameter(PK_Param, #X, DefaultValue); \
    (void)Var;
#define _LOCALPARAM(Var, X, DefaultValue) \
    VerilogParameter *Var = Module->CreateParameter(PK_LocalParam, #X, \
                                                    DefaultValue); \
    (void)Var;
#define _INPUT(X) \
    Module->CreateWire(SP_Input, #X);
#define _INPUTV(X, VWidth) \
    Module->CreateWire(SP_Input, #X, VerilogParamConst(VWidth));
#define _INPUTA(X, VWidth, AWidth) \
    Module->CreateWire(SP_Input, #X, VerilogParamConst(VWidth), \
                       VerilogParamConst(AWidth));
#define _OUTPUT(X) \
    Module->CreateWire(SP_Output, #X);
#define _OUTPUTV(X, VWidth) \
    Module->CreateWire(SP_Output, #X, VerilogParamConst(VWidth));
#define _OUTPUTA(X, VWidth, AWidth) \
    Module->CreateWire(SP_Output, #X, VerilogParamConst(VWidth), \
                       VerilogParamConst(AWidth));
#define _END_MODULE(X) \
    IP[#X] = new PlatformIPCore(Module); }

  _MODULE(fifo_queue)
    _PARAM(Width, WIDTH, 1)
    _PARAM(MaxLatency, MAX_LATENCY, 1)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTV(in, Width)
    _INPUT(valid_in)
    _OUTPUT(wait_in)
    _OUTPUTV(out, Width)
    _OUTPUT(valid_out)
    _INPUT(wait_out)
  _END_MODULE(fifo_queue)

  _MODULE(dc_data_width_adapter)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(MasterDataWidth, MASTER_DATA_WIDTH, 512)
    _LOCALPARAM(MasterDataWidthByte, MASTER_DATA_WIDTH_BYTE, 64)
    _PARAM(SlaveDataWidth, SLAVE_DATA_WIDTH, 512)
    _LOCALPARAM(SlaveDataWidthByte, SLAVE_DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_read)
    _OUTPUTV(avs_readdata, SlaveDataWidth)
    _OUTPUT(avs_readdatavalid)
    _INPUT(avs_write)
    _INPUTV(avs_writedata, SlaveDataWidth)
    _INPUTV(avs_byteenable, SlaveDataWidthByte)
    _OUTPUT(avs_waitrequest)
    _INPUT(avs_waitresponse)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, MasterDataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, MasterDataWidth)
    _OUTPUTV(avm_byteenable, MasterDataWidthByte)
    _INPUT(avm_waitrequest)
    _OUTPUT(avm_waitresponse)
  _END_MODULE(dc_data_width_adapter)

  _MODULE(dc_data_width_adapter_ro)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(MasterDataWidth, MASTER_DATA_WIDTH, 512)
    _LOCALPARAM(MasterDataWidthByte, MASTER_DATA_WIDTH_BYTE, 64)
    _PARAM(SlaveDataWidth, SLAVE_DATA_WIDTH, 512)
    _LOCALPARAM(SlaveDataWidthByte, SLAVE_DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_read)
    _OUTPUTV(avs_readdata, SlaveDataWidth)
    _OUTPUT(avs_readdatavalid)
    _OUTPUT(avs_waitrequest)
    _INPUT(avs_waitresponse)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, MasterDataWidth)
    _INPUT(avm_readdatavalid)
    _INPUT(avm_waitrequest)
    _OUTPUT(avm_waitresponse)
  _END_MODULE(dc_data_width_adapter_ro)

  _MODULE(dc_data_width_adapter_wo)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(MasterDataWidth, MASTER_DATA_WIDTH, 512)
    _LOCALPARAM(MasterDataWidthByte, MASTER_DATA_WIDTH_BYTE, 64)
    _PARAM(SlaveDataWidth, SLAVE_DATA_WIDTH, 512)
    _LOCALPARAM(SlaveDataWidthByte, SLAVE_DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_write)
    _INPUTV(avs_writedata, SlaveDataWidth)
    _INPUTV(avs_byteenable, SlaveDataWidthByte)
    _OUTPUT(avs_waitrequest)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, MasterDataWidth)
    _OUTPUTV(avm_byteenable, MasterDataWidthByte)
    _INPUT(avm_waitrequest)
  _END_MODULE(dc_data_width_adapter_wo)

  _MODULE(dc_arbiter_n1)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_read, 1, NumMasters)
    _OUTPUTA(avs_readdata, DataWidth, NumMasters)
    _OUTPUTA(avs_readdatavalid, 1, NumMasters)
    _INPUTA(avs_write, 1, NumMasters)
    _INPUTA(avs_writedata, DataWidth, NumMasters)
    _INPUTA(avs_byteenable, DataWidthByte, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _INPUTA(avs_waitresponse, 1, NumMasters)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _INPUT(avm_waitrequest)
    _OUTPUT(avm_waitresponse)
  _END_MODULE(dc_arbiter_n1)

  _MODULE(dc_arbiter_n1_ro)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_read, 1, NumMasters)
    _OUTPUTA(avs_readdata, DataWidth, NumMasters)
    _OUTPUTA(avs_readdatavalid, 1, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _INPUTA(avs_waitresponse, 1, NumMasters)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _INPUT(avm_waitrequest)
    _OUTPUT(avm_waitresponse)
  _END_MODULE(dc_arbiter_n1_ro)

  _MODULE(dc_arbiter_n1_wo)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_write, 1, NumMasters)
    _INPUTA(avs_writedata, DataWidth, NumMasters)
    _INPUTA(avs_byteenable, DataWidthByte, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _INPUT(avm_waitrequest)
  _END_MODULE(dc_arbiter_n1_wo)

  _MODULE(dc_arbiter_nm)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(NumSlaves, NUM_SLAVES, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(NumSlavesWidth, NUM_SLAVES_WIDTH, 2)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_work_group_id, NumSlavesWidth, NumMasters)
    _INPUTA(avs_read, 1, NumMasters)
    _OUTPUTA(avs_readdata, DataWidth, NumMasters)
    _OUTPUTA(avs_readdatavalid, 1, NumMasters)
    _INPUTA(avs_write, 1, NumMasters)
    _INPUTA(avs_writedata, DataWidth, NumMasters)
    _INPUTA(avs_byteenable, DataWidthByte, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _INPUTA(avs_waitresponse, 1, NumMasters)
    _OUTPUTA(avm_address, AddrWidth, NumSlaves)
    _OUTPUTA(avm_read, 1, NumSlaves)
    _INPUTA(avm_readdata, DataWidth, NumSlaves)
    _INPUTA(avm_readdatavalid, 1, NumSlaves)
    _OUTPUTA(avm_write, 1, NumSlaves)
    _OUTPUTA(avm_writedata, DataWidth, NumSlaves)
    _OUTPUTA(avm_byteenable, DataWidthByte, NumSlaves)
    _INPUTA(avm_waitrequest, 1, NumSlaves)
    _OUTPUTA(avm_waitresponse, 1, NumSlaves)
  _END_MODULE(dc_arbiter_nm)

  _MODULE(dc_arbiter_nm_ro)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(NumSlaves, NUM_SLAVES, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(NumSlavesWidth, NUM_SLAVES_WIDTH, 2)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_work_group_id, NumSlavesWidth, NumMasters)
    _INPUTA(avs_read, 1, NumMasters)
    _OUTPUTA(avs_readdata, DataWidth, NumMasters)
    _OUTPUTA(avs_readdatavalid, 1, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _INPUTA(avs_waitresponse, 1, NumMasters)
    _OUTPUTA(avm_address, AddrWidth, NumSlaves)
    _OUTPUTA(avm_read, 1, NumSlaves)
    _INPUTA(avm_readdata, DataWidth, NumSlaves)
    _INPUTA(avm_readdatavalid, 1, NumSlaves)
    _INPUTA(avm_waitrequest, 1, NumSlaves)
    _OUTPUTA(avm_waitresponse, 1, NumSlaves)
  _END_MODULE(dc_arbiter_nm_ro)

  _MODULE(dc_arbiter_nm_wo)
    _PARAM(NumMasters, NUM_MASTERS, 2)
    _PARAM(NumSlaves, NUM_SLAVES, 2)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(NumSlavesWidth, NUM_SLAVES_WIDTH, 2)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_work_group_id, NumSlavesWidth, NumMasters)
    _INPUTA(avs_write, 1, NumMasters)
    _INPUTA(avs_writedata, DataWidth, NumMasters)
    _INPUTA(avs_byteenable, DataWidthByte, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _OUTPUTA(avm_address, AddrWidth, NumSlaves)
    _OUTPUTA(avm_write, 1, NumSlaves)
    _OUTPUTA(avm_writedata, DataWidth, NumSlaves)
    _OUTPUTA(avm_byteenable, DataWidthByte, NumSlaves)
    _INPUTA(avm_waitrequest, 1, NumSlaves)
  _END_MODULE(dc_arbiter_nm_wo)

  if (PVars.GetVar("mem", "use_axi")) {
    _MODULE(mem_arbiter_41)
      _INPUT(clk)
      _INPUT(rstn)

      _OUTPUTV(mem0_araddr, 48)
      _OUTPUTV(mem0_arburst, 2)
      _OUTPUTV(mem0_arcache, 4)
      _OUTPUTV(mem0_arlen, 8)
      _INPUT(mem0_arready)
      _OUTPUTV(mem0_arsize, 3)
      _OUTPUT(mem0_arvalid)
      _OUTPUTV(mem0_awaddr, 48)
      _OUTPUTV(mem0_awburst, 2)
      _OUTPUTV(mem0_awcache, 4)
      _OUTPUTV(mem0_awlen, 8)
      _INPUT(mem0_awready)
      _OUTPUTV(mem0_awsize, 3)
      _OUTPUT(mem0_awvalid)
      _OUTPUT(mem0_bready)
      _INPUTV(mem0_bresp, 2)
      _INPUT(mem0_bvalid)
      _INPUTV(mem0_rdata, 512)
      _INPUT(mem0_rlast)
      _OUTPUT(mem0_rready)
      _INPUTV(mem0_rresp, 2)
      _INPUT(mem0_rvalid)
      _OUTPUTV(mem0_wdata, 512)
      _OUTPUT(mem0_wlast)
      _INPUT(mem0_wready)
      _OUTPUTV(mem0_wstrb, 64)
      _OUTPUT(mem0_wvalid)

      _INPUTV(mem1_araddr, 48)
      _INPUTV(mem1_arburst, 2)
      _INPUTV(mem1_arcache, 4)
      _INPUTV(mem1_arlen, 8)
      _OUTPUT(mem1_arready)
      _INPUTV(mem1_arsize, 3)
      _INPUT(mem1_arvalid)
      _INPUTV(mem1_awaddr, 48)
      _INPUTV(mem1_awburst, 2)
      _INPUTV(mem1_awcache, 4)
      _INPUTV(mem1_awlen, 8)
      _OUTPUT(mem1_awready)
      _INPUTV(mem1_awsize, 3)
      _INPUT(mem1_awvalid)
      _INPUT(mem1_bready)
      _OUTPUTV(mem1_bresp, 2)
      _OUTPUT(mem1_bvalid)
      _OUTPUTV(mem1_rdata, 512)
      _OUTPUT(mem1_rlast)
      _INPUT(mem1_rready)
      _OUTPUTV(mem1_rresp, 2)
      _OUTPUT(mem1_rvalid)
      _INPUTV(mem1_wdata, 512)
      _INPUT(mem1_wlast)
      _OUTPUT(mem1_wready)
      _INPUTV(mem1_wstrb, 64)
      _INPUT(mem1_wvalid)

      _INPUTV(mem2_araddr, 48)
      _INPUTV(mem2_arburst, 2)
      _INPUTV(mem2_arcache, 4)
      _INPUTV(mem2_arlen, 8)
      _OUTPUT(mem2_arready)
      _INPUTV(mem2_arsize, 3)
      _INPUT(mem2_arvalid)
      _INPUTV(mem2_awaddr, 48)
      _INPUTV(mem2_awburst, 2)
      _INPUTV(mem2_awcache, 4)
      _INPUTV(mem2_awlen, 8)
      _OUTPUT(mem2_awready)
      _INPUTV(mem2_awsize, 3)
      _INPUT(mem2_awvalid)
      _INPUT(mem2_bready)
      _OUTPUTV(mem2_bresp, 2)
      _OUTPUT(mem2_bvalid)
      _OUTPUTV(mem2_rdata, 512)
      _OUTPUT(mem2_rlast)
      _INPUT(mem2_rready)
      _OUTPUTV(mem2_rresp, 2)
      _OUTPUT(mem2_rvalid)
      _INPUTV(mem2_wdata, 512)
      _INPUT(mem2_wlast)
      _OUTPUT(mem2_wready)
      _INPUTV(mem2_wstrb, 64)
      _INPUT(mem2_wvalid)

      _INPUTV(mem3_araddr, 48)
      _INPUTV(mem3_arburst, 2)
      _INPUTV(mem3_arcache, 4)
      _INPUTV(mem3_arlen, 8)
      _OUTPUT(mem3_arready)
      _INPUTV(mem3_arsize, 3)
      _INPUT(mem3_arvalid)
      _INPUTV(mem3_awaddr, 48)
      _INPUTV(mem3_awburst, 2)
      _INPUTV(mem3_awcache, 4)
      _INPUTV(mem3_awlen, 8)
      _OUTPUT(mem3_awready)
      _INPUTV(mem3_awsize, 3)
      _INPUT(mem3_awvalid)
      _INPUT(mem3_bready)
      _OUTPUTV(mem3_bresp, 2)
      _OUTPUT(mem3_bvalid)
      _OUTPUTV(mem3_rdata, 512)
      _OUTPUT(mem3_rlast)
      _INPUT(mem3_rready)
      _OUTPUTV(mem3_rresp, 2)
      _OUTPUT(mem3_rvalid)
      _INPUTV(mem3_wdata, 512)
      _INPUT(mem3_wlast)
      _OUTPUT(mem3_wready)
      _INPUTV(mem3_wstrb, 64)
      _INPUT(mem3_wvalid)

      _INPUTV(mem4_araddr, 48)
      _INPUTV(mem4_arburst, 2)
      _INPUTV(mem4_arcache, 4)
      _INPUTV(mem4_arlen, 8)
      _OUTPUT(mem4_arready)
      _INPUTV(mem4_arsize, 3)
      _INPUT(mem4_arvalid)
      _INPUTV(mem4_awaddr, 48)
      _INPUTV(mem4_awburst, 2)
      _INPUTV(mem4_awcache, 4)
      _INPUTV(mem4_awlen, 8)
      _OUTPUT(mem4_awready)
      _INPUTV(mem4_awsize, 3)
      _INPUT(mem4_awvalid)
      _INPUT(mem4_bready)
      _OUTPUTV(mem4_bresp, 2)
      _OUTPUT(mem4_bvalid)
      _OUTPUTV(mem4_rdata, 512)
      _OUTPUT(mem4_rlast)
      _INPUT(mem4_rready)
      _OUTPUTV(mem4_rresp, 2)
      _OUTPUT(mem4_rvalid)
      _INPUTV(mem4_wdata, 512)
      _INPUT(mem4_wlast)
      _OUTPUT(mem4_wready)
      _INPUTV(mem4_wstrb, 64)
      _INPUT(mem4_wvalid)
    _END_MODULE(mem_arbiter_41)

    _MODULE(amm_to_axi_converter_nonburst)
      _PARAM(AddrWidth, ADDR_WIDTH, 48)
      _PARAM(DataWidth, DATA_WIDTH, 512)
      _PARAM(BurstWidth, BURST_WIDTH, 4)
      _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
      _INPUT(clk)
      _INPUT(rstn)
      _INPUTV(amm_address, AddrWidth)
      _INPUT(amm_read)
      _OUTPUTV(amm_readdata, DataWidth)
      _OUTPUT(amm_readdatavalid)
      _INPUT(amm_write)
      _INPUTV(amm_writedata, DataWidth)
      _INPUTV(amm_byteenable, DataWidthByte)
      _INPUTV(amm_burstcount, BurstWidth)
      _OUTPUT(amm_waitrequest)
      _OUTPUTV(axi_araddr, AddrWidth)
      _OUTPUTV(axi_arburst, 2)
      _OUTPUTV(axi_arcache, 4)
      _OUTPUTV(axi_arlen, 8)
      _INPUT(axi_arready)
      _OUTPUTV(axi_arsize, 3)
      _OUTPUT(axi_arvalid)
      _OUTPUTV(axi_awaddr, AddrWidth)
      _OUTPUTV(axi_awburst, 2)
      _OUTPUTV(axi_awcache, 4)
      _OUTPUTV(axi_awlen, 8)
      _INPUT(axi_awready)
      _OUTPUTV(axi_awsize, 3)
      _OUTPUT(axi_awvalid)
      _OUTPUT(axi_bready)
      _INPUTV(axi_bresp, 2)
      _INPUT(axi_bvalid)
      _INPUTV(axi_rdata, DataWidth)
      _INPUT(axi_rlast)
      _OUTPUT(axi_rready)
      _INPUTV(axi_rresp, 2)
      _INPUT(axi_rvalid)
      _OUTPUTV(axi_wdata, DataWidth)
      _OUTPUT(axi_wlast)
      _INPUT(axi_wready)
      _OUTPUTV(axi_wstrb, DataWidthByte)
      _OUTPUT(axi_wvalid)
    _END_MODULE(amm_to_axi_converter_nonburst)

  } else {
    _MODULE(mem_arbiter_41)
      _INPUT(clk)
      _INPUT(rstn)
      _INPUTV(avs0_address, 48)
      _INPUTV(avs0_burstcount, 4)
      _OUTPUT(avs0_waitrequest)
      _INPUT(avs0_read)
      _OUTPUTV(avs0_readdata, 512)
      _OUTPUT(avs0_readdatavalid)
      _INPUT(avs0_write)
      _INPUTV(avs0_writedata, 512)
      _INPUTV(avs0_byteenable, 64)
      _INPUTV(avs1_address, 48)
      _INPUTV(avs1_burstcount, 4)
      _OUTPUT(avs1_waitrequest)
      _INPUT(avs1_read)
      _OUTPUTV(avs1_readdata, 512)
      _OUTPUT(avs1_readdatavalid)
      _INPUT(avs1_write)
      _INPUTV(avs1_writedata, 512)
      _INPUTV(avs1_byteenable, 64)
      _INPUTV(avs2_address, 48)
      _INPUTV(avs2_burstcount, 4)
      _OUTPUT(avs2_waitrequest)
      _INPUT(avs2_read)
      _OUTPUTV(avs2_readdata, 512)
      _OUTPUT(avs2_readdatavalid)
      _INPUT(avs2_write)
      _INPUTV(avs2_writedata, 512)
      _INPUTV(avs2_byteenable, 64)
      _INPUTV(avs3_address, 48)
      _INPUTV(avs3_burstcount, 4)
      _OUTPUT(avs3_waitrequest)
      _INPUT(avs3_read)
      _OUTPUTV(avs3_readdata, 512)
      _OUTPUT(avs3_readdatavalid)
      _INPUT(avs3_write)
      _INPUTV(avs3_writedata, 512)
      _INPUTV(avs3_byteenable, 64)
      _OUTPUTV(avm_address, 48)
      _OUTPUTV(avm_burstcount, 4)
      _INPUT(avm_waitrequest)
      _OUTPUT(avm_read)
      _INPUTV(avm_readdata, 512)
      _INPUT(avm_readdatavalid)
      _OUTPUT(avm_write)
      _OUTPUTV(avm_writedata, 512)
      _OUTPUTV(avm_byteenable, 64)
    _END_MODULE(mem_arbiter_41)
  }

  _MODULE(global_mem_bypass)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _PARAM(BurstWidth, BURST_WIDTH, 4)
    _PARAM(DatapathDataWidth, DATAPATH_DATA_WIDTH, 512)
    _LOCALPARAM(DatapathDataWidthByte, DATAPATH_DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _OUTPUTV(avm_burstcount, BurstWidth)
    _INPUT(avm_waitrequest)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_read)
    _OUTPUTV(avs_readdata, DatapathDataWidth)
    _OUTPUT(avs_readdatavalid)
    _INPUT(avs_write)
    _INPUTV(avs_writedata, DatapathDataWidth)
    _INPUTV(avs_byteenable, DatapathDataWidthByte)
    _OUTPUT(avs_waitrequest)
    _INPUT(avs_waitresponse)
  _END_MODULE(global_mem_bypass)

  _MODULE(global_mem_cache)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _LOCALPARAM(BurstWidth, BURST_WIDTH, 4)
    _PARAM(DatapathDataWidth, DATAPATH_DATA_WIDTH, 512)
    _LOCALPARAM(DatapathDataWidthByte, DATAPATH_DATA_WIDTH_BYTE, 64)
    _PARAM(DistributionFactor, DISTRIBUTION_FACTOR, 1)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUT(cache_clean)
    _OUTPUT(cache_cleaned)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _INPUT(avm_waitrequest)
    _OUTPUTV(avm_burstcount, BurstWidth)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_read)
    _OUTPUTV(avs_readdata, DatapathDataWidth)
    _OUTPUT(avs_readdatavalid)
    _INPUT(avs_write)
    _INPUTV(avs_writedata, DatapathDataWidth)
    _INPUTV(avs_byteenable, DatapathDataWidthByte)
    _OUTPUT(avs_waitrequest)
    _INPUT(avs_waitresponse)
  _END_MODULE(global_mem_cache)

  _MODULE(global_mem_cache_ro)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _LOCALPARAM(BurstWidth, BURST_WIDTH, 4)
    _PARAM(DatapathDataWidth, DATAPATH_DATA_WIDTH, 512)
    _LOCALPARAM(DatapathDataWidthByte, DATAPATH_DATA_WIDTH_BYTE, 64)
    _PARAM(DistributionFactor, DISTRIBUTION_FACTOR, 1)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUT(cache_clean)
    _OUTPUT(cache_cleaned)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _INPUT(avm_waitrequest)
    _OUTPUTV(avm_burstcount, BurstWidth)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_read)
    _OUTPUTV(avs_readdata, DatapathDataWidth)
    _OUTPUT(avs_readdatavalid)
    _OUTPUT(avs_waitrequest)
    _INPUT(avs_waitresponse)
  _END_MODULE(global_mem_cache_ro)

  _MODULE(global_mem_cache_wo)
    _PARAM(AddrWidth, ADDR_WIDTH, 48)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _LOCALPARAM(BurstWidth, BURST_WIDTH, 4)
    _PARAM(DatapathDataWidth, DATAPATH_DATA_WIDTH, 512)
    _LOCALPARAM(DatapathDataWidthByte, DATAPATH_DATA_WIDTH_BYTE, 64)
    _PARAM(DistributionFactor, DISTRIBUTION_FACTOR, 1)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUT(cache_clean)
    _OUTPUT(cache_cleaned)
    _OUTPUTV(avm_address, AddrWidth)
    _OUTPUT(avm_read)
    _INPUTV(avm_readdata, DataWidth)
    _INPUT(avm_readdatavalid)
    _OUTPUT(avm_write)
    _OUTPUTV(avm_writedata, DataWidth)
    _OUTPUTV(avm_byteenable, DataWidthByte)
    _INPUT(avm_waitrequest)
    _OUTPUTV(avm_burstcount, BurstWidth)
    _INPUTV(avs_address, AddrWidth)
    _INPUT(avs_write)
    _INPUTV(avs_writedata, DatapathDataWidth)
    _INPUTV(avs_byteenable, DatapathDataWidthByte)
    _OUTPUT(avs_waitrequest)
  _END_MODULE(global_mem_cache_wo)

  _MODULE(local_memory)
    _PARAM(NumMasters, NUM_MASTERS, 1)
    _PARAM(AddrWidth, ADDR_WIDTH, 16)
    _PARAM(DataWidth, DATA_WIDTH, 512)
    _PARAM(BankWidth, BANK_WIDTH, 2)
    _LOCALPARAM(DataWidthByte, DATA_WIDTH_BYTE, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(avs_address, AddrWidth, NumMasters)
    _INPUTA(avs_read, 1, NumMasters)
    _OUTPUTA(avs_readdata, DataWidth, NumMasters)
    _OUTPUTA(avs_readdatavalid, 1, NumMasters)
    _INPUTA(avs_write, 1, NumMasters)
    _INPUTA(avs_writedata, DataWidth, NumMasters)
    _INPUTA(avs_byteenable, DataWidthByte, NumMasters)
    _OUTPUTA(avs_waitrequest, 1, NumMasters)
    _INPUTA(avs_waitresponse, 1, NumMasters)
  _END_MODULE(local_memory)

  _MODULE(lock)
    _PARAM(NumMasters, NUM_MASTERS, 4)
    _PARAM(LockWidth, LOCK_WIDTH, 4)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTA(bys_lockid, LockWidth, NumMasters)
    _INPUTA(bys_lockrequestvalid, 1, NumMasters)
    _OUTPUTA(bys_lockwaitrequest, 1, NumMasters)
    _OUTPUTA(bys_lockresponsevalid, 1, NumMasters)
    _INPUTA(bys_lockwaitresponse, 1, NumMasters)
    _INPUTA(bys_unlockid, LockWidth, NumMasters)
    _INPUTA(bys_unlockrequestvalid, 1, NumMasters)
    _OUTPUTA(bys_unlockwaitrequest, 1, NumMasters)
    _OUTPUTA(bys_unlockresponsevalid, 1, NumMasters)
    _INPUTA(bys_unlockwaitresponse, 1, NumMasters)
  _END_MODULE(lock)

  _MODULE(work_group_dispatcher)
    _PARAM(NumOutput, NUM_OUTPUT, 1)
    _PARAM(GWidth, GWIDTH, 32)
    _PARAM(LWidth, LWIDTH, 9)
    _PARAM(NWidth, NWIDTH, 64)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUT(trigger)
    _INPUTV(num_groups_0, GWidth)
    _INPUTV(num_groups_1, GWidth)
    _INPUTV(num_groups_2, GWidth)
    _INPUTV(local_size_0, LWidth + 1)
    _INPUTV(local_size_1, LWidth + 1)
    _INPUTV(local_size_2, LWidth + 1)
    _OUTPUTA(group_id_0, GWidth, NumOutput)
    _OUTPUTA(group_id_1, GWidth, NumOutput)
    _OUTPUTA(group_id_2, GWidth, NumOutput)
    _OUTPUTA(global_offset_0, GWidth, NumOutput)
    _OUTPUTA(global_offset_1, GWidth, NumOutput)
    _OUTPUTA(global_offset_2, GWidth, NumOutput)
    _OUTPUTA(flat_group_id, NWidth, NumOutput)
    _OUTPUTA(valid_out, 1, NumOutput)
    _INPUTA(wait_out, 1, NumOutput)
  _END_MODULE(work_group_dispatcher)

  _MODULE(work_item_dispatcher)
    _PARAM(GWidth, GWIDTH, 32)
    _PARAM(LWidth, LWIDTH, 9)
    _INPUT(clk)
    _INPUT(rstn)
    _INPUTV(local_size_0, LWidth + 1)
    _INPUTV(local_size_1, LWidth + 1)
    _INPUTV(local_size_2, LWidth + 1)
    _INPUTV(global_offset_0, GWidth)
    _INPUTV(global_offset_1, GWidth)
    _INPUTV(global_offset_2, GWidth)
    _INPUT(valid_in)
    _OUTPUT(wait_in)
    _OUTPUTV(global_id_0, GWidth)
    _OUTPUTV(global_id_1, GWidth)
    _OUTPUTV(global_id_2, GWidth)
    _OUTPUTV(local_id_0, LWidth)
    _OUTPUTV(local_id_1, LWidth)
    _OUTPUTV(local_id_2, LWidth)
    _OUTPUTV(flat_local_id, LWidth)
    _OUTPUT(valid_out)
    _INPUT(wait_out)
  _END_MODULE(work_item_dispatcher)

#undef _MODULE
#undef _PARAM
#undef _LOCALPARAM
#undef _INPUT
#undef _INPUTV
#undef _INPUTA
#undef _OUTPUT
#undef _OUTPUTV
#undef _OUTPUTA
#undef _END_MODULE
}

PlatformIPCore *PlatformIPCollection::getIP(StringRef Name) {
  assert(IP.count(Name) && "IP not found");
  return IP[Name];
}

PlatformIPCore *PlatformIPCollection::getIP(DFGUnaryOpNode *Op) {
  QualType FromType = Op->getOperand()->getType();
  QualType ToType = Op->getType();

  switch (Op->getOpcode()) {
    case DFGUnaryOpNode::DUO_Minus:
      return getIP("minus_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Not:
      return NULL;

    case DFGUnaryOpNode::DUO_LNot:
      return getIP("lnot_" + GetTypeCode(FromType));
    case DFGUnaryOpNode::DUO_LNot_V:
      return getIP("lnot_v_" + GetTypeCode(FromType));

    case DFGUnaryOpNode::DUO_ReinterpretCast:
      return NULL;

    case DFGUnaryOpNode::DUO_IntCast:
      return NULL;

    case DFGUnaryOpNode::DUO_IntToFloatCast: {
      unsigned ToSize = (unsigned)ASTCtx.getTypeSize(ToType);
      if (ToSize == 32) {
        return getIP("itof_" + GetTypeCode(FromType));
      } else if (ToSize == 64) {
        return getIP("itod_" + GetTypeCode(FromType));
      } else {
        return NULL;
      }
    }
    case DFGUnaryOpNode::DUO_FloatToIntCast: {
      unsigned FromSize = (unsigned)ASTCtx.getTypeSize(FromType);
      if (FromSize == 32) {
        return getIP("ftoi_" + GetTypeCode(ToType));
      } else if (FromSize == 64) {
        return getIP("dtoi_" + GetTypeCode(ToType));
      } else {
        return NULL;
      }
    }
    case DFGUnaryOpNode::DUO_FloatCast: {
      unsigned FromSize = (unsigned)ASTCtx.getTypeSize(FromType);
      unsigned ToSize = (unsigned)ASTCtx.getTypeSize(ToType);
      if (FromSize == 32 && ToSize == 64) {
        return getIP("ftod");
      } else if (FromSize == 64 && ToSize == 32) {
        return getIP("dtof");
      } else {
        return NULL;
      }
    }
    case DFGUnaryOpNode::DUO_Cos:
      return getIP("cos_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Sin:
      return getIP("sin_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Tan:
      return getIP("tan_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Acos:
      return getIP("acos_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Asin:
      return getIP("asin_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Atan:
      return getIP("atan_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Exp:
      return getIP("exp_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Log:
      return getIP("log_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Log2:
      return getIP("log2_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Log10:
      return getIP("log10_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Sqrt:
      return getIP("sqrt_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_RSqrt:
      return getIP("rsqrt_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Ceil:
      return getIP("ceil_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Floor:
      return getIP("floor_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Round:
      return getIP("round_" + GetTypeCode(ToType));
    case DFGUnaryOpNode::DUO_Trunc:
      return getIP("trunc_" + GetTypeCode(ToType));

    case DFGUnaryOpNode::DUO_Abs: {
      if (FromType->isUnsignedIntegerType()) {
        return NULL;
      } else {
        return getIP("abs_" + GetTypeCode(FromType));
      }
    }
  }
  return NULL;
}

PlatformIPCore *PlatformIPCollection::getIP(DFGBinaryOpNode *Op) {
  QualType FromType = Op->getLHS()->getType();
  QualType ToType = Op->getType();

  switch (Op->getOpcode()) {
    case DFGBinaryOpNode::DBO_Mul:
      return getIP("mul_" + GetTypeCode(ToType));
    case DFGBinaryOpNode::DBO_Div:
      return getIP("div_" + GetTypeCode(ToType));
    case DFGBinaryOpNode::DBO_Rem:
      return getIP("rem_" + GetTypeCode(ToType));
    case DFGBinaryOpNode::DBO_Add:
      return getIP("add_" + GetTypeCode(ToType));
    case DFGBinaryOpNode::DBO_Sub:
      return getIP("sub_" + GetTypeCode(ToType));

    case DFGBinaryOpNode::DBO_Shl:
      return getIP("shl_" + GetTypeCode(ToType));
    case DFGBinaryOpNode::DBO_Shr:
      return getIP("shr_" + GetTypeCode(ToType));

    case DFGBinaryOpNode::DBO_LT:
      return getIP("cmp_lt_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_GT:
      return getIP("cmp_gt_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_LE:
      return getIP("cmp_le_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_GE:
      return getIP("cmp_ge_" + GetTypeCode(FromType));

    case DFGBinaryOpNode::DBO_LT_V:
      return getIP("cmp_lt_v_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_GT_V:
      return getIP("cmp_gt_v_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_LE_V:
      return getIP("cmp_le_v_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_GE_V:
      return getIP("cmp_ge_v_" + GetTypeCode(FromType));

    case DFGBinaryOpNode::DBO_EQ:
      return getIP("cmp_eq_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_NE:
      return getIP("cmp_ne_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_EQ_V:
      return getIP("cmp_eq_v_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_NE_V:
      return getIP("cmp_ne_v_" + GetTypeCode(FromType));

    case DFGBinaryOpNode::DBO_And:
    case DFGBinaryOpNode::DBO_Xor:
    case DFGBinaryOpNode::DBO_Or:
      return NULL;

    case DFGBinaryOpNode::DBO_LAnd:
      return getIP("land_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_LAnd_V:
      return getIP("land_v_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_LOr:
      return getIP("lor_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_LOr_V:
      return getIP("lor_v_" + GetTypeCode(FromType));

    case DFGBinaryOpNode::DBO_Pow:
      return getIP("pow_" + GetTypeCode(ToType));

    case DFGBinaryOpNode::DBO_Fmod:
      return getIP("fmod_" + GetTypeCode(ToType));

    case DFGBinaryOpNode::DBO_Max:
      return getIP("max_" + GetTypeCode(FromType));
    case DFGBinaryOpNode::DBO_Min:
      return getIP("min_" + GetTypeCode(FromType));

    case DFGBinaryOpNode::DBO_Mul24:
      return getIP("mul24_" + GetTypeCode(ToType));
  }
  return NULL;
}

PlatformIPCore *PlatformIPCollection::getIP(DFGTernaryOpNode *Op) {
  return NULL;
}

PlatformIPCore *PlatformIPCollection::getIP(DFGNullaryAtomicNode *Op) {
  QualType ToType = Op->getType();

  switch (Op->getOpcode()) {
    case DFGNullaryAtomicNode::DNA_Inc:
      return getIP("atomic_inc_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGNullaryAtomicNode::DNA_Dec:
      return getIP("atomic_dec_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
  }
  return NULL;
}

PlatformIPCore *PlatformIPCollection::getIP(DFGUnaryAtomicNode *Op) {
  QualType ToType = Op->getType();

  switch (Op->getOpcode()) {
    case DFGUnaryAtomicNode::DUA_Add:
      return getIP("atomic_add_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Sub:
      return getIP("atomic_sub_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Xchg:
      return getIP("atomic_xchg_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Min:
      return getIP("atomic_min_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Max:
      return getIP("atomic_max_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_And:
      return getIP("atomic_and_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Or:
      return getIP("atomic_or_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
    case DFGUnaryAtomicNode::DUA_Xor:
      return getIP("atomic_xor_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
  }
  return NULL;
}

PlatformIPCore *PlatformIPCollection::getIP(DFGBinaryAtomicNode *Op) {
  QualType ToType = Op->getType();

  switch (Op->getOpcode()) {
    case DFGBinaryAtomicNode::DBA_CmpXchg:
      return getIP("atomic_cmpxchg_" + GetAddressSpaceCode(Op->getAddressSpace()) +
                   "_" + GetTypeCode(ToType));
  }
  return NULL;
}

void PlatformIPCollection::AddUnaryOpIP(StringRef name, unsigned aWidth,
                                        unsigned qWidth) {
  VerilogModule *Op = new VerilogModule(name);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "enable");
  Op->CreateWire(SP_Input, "a", VerilogParamConst(aWidth));
  Op->CreateWire(SP_Output, "q", VerilogParamConst(qWidth));
  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

void PlatformIPCollection::AddBinaryOpIP(StringRef name, unsigned aWidth,
                                         unsigned bWidth, unsigned qWidth) {
  VerilogModule *Op = new VerilogModule(name);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "enable");
  Op->CreateWire(SP_Input, "a", VerilogParamConst(aWidth));
  Op->CreateWire(SP_Input, "b", VerilogParamConst(bWidth));
  Op->CreateWire(SP_Output, "q", VerilogParamConst(qWidth));
  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

void PlatformIPCollection::AddTernaryOpIP(StringRef name, unsigned aWidth,
                                          unsigned bWidth, unsigned cWidth,
                                          unsigned qWidth) {
  VerilogModule *Op = new VerilogModule(name);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "enable");
  Op->CreateWire(SP_Input, "a", VerilogParamConst(aWidth));
  Op->CreateWire(SP_Input, "b", VerilogParamConst(bWidth));
  Op->CreateWire(SP_Input, "c", VerilogParamConst(cWidth));
  Op->CreateWire(SP_Output, "q", VerilogParamConst(qWidth));
  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

void PlatformIPCollection::AddNullaryAtomicIP(StringRef name, unsigned width) {
  VerilogModule *Op = new VerilogModule(name);
  VerilogParameter *AddrWidth = Op->CreateParameter(PK_Param, "ADDR_WIDTH", 48);
  VerilogParameter *DataWidth = Op->CreateParameter(PK_Param, "DATA_WIDTH", 512);
  VerilogParameter *LockWidth = Op->CreateParameter(PK_Param, "LOCK_WIDTH", 4);
  VerilogParameter *DataWidthByte = Op->CreateParameter(PK_LocalParam,
                                                        "DATA_WIDTH_BYTE", 64);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "addr", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Input, "valid_in");
  Op->CreateWire(SP_Output, "wait_in");
  Op->CreateWire(SP_Output, "q", VerilogParamConst(width));
  Op->CreateWire(SP_Output, "valid_out");
  Op->CreateWire(SP_Input, "wait_out");

  Op->CreateWire(SP_Output, "bym_lockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_lockrequestvalid");
  Op->CreateWire(SP_Input, "bym_lockwaitrequest");
  Op->CreateWire(SP_Input, "bym_lockresponsevalid");
  Op->CreateWire(SP_Output, "bym_lockwaitresponse");
  Op->CreateWire(SP_Output, "bym_unlockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_unlockrequestvalid");
  Op->CreateWire(SP_Input, "bym_unlockwaitrequest");
  Op->CreateWire(SP_Input, "bym_unlockresponsevalid");
  Op->CreateWire(SP_Output, "bym_unlockwaitresponse");

  Op->CreateWire(SP_Output, "avm_address", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Output, "avm_read");
  Op->CreateWire(SP_Input, "avm_readdata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Input, "avm_readdatavalid");
  Op->CreateWire(SP_Output, "avm_write");
  Op->CreateWire(SP_Output, "avm_writedata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Output, "avm_byteenable", VerilogParamConst(DataWidthByte));
  Op->CreateWire(SP_Input, "avm_waitrequest");
  Op->CreateWire(SP_Output, "avm_waitresponse");

  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

void PlatformIPCollection::AddUnaryAtomicIP(StringRef name, unsigned width) {
  VerilogModule *Op = new VerilogModule(name);
  VerilogParameter *AddrWidth = Op->CreateParameter(PK_Param, "ADDR_WIDTH", 48);
  VerilogParameter *DataWidth = Op->CreateParameter(PK_Param, "DATA_WIDTH", 512);
  VerilogParameter *LockWidth = Op->CreateParameter(PK_Param, "LOCK_WIDTH", 4);
  VerilogParameter *DataWidthByte = Op->CreateParameter(PK_LocalParam,
                                                        "DATA_WIDTH_BYTE", 64);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "addr", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Input, "a", VerilogParamConst(width));
  Op->CreateWire(SP_Input, "valid_in");
  Op->CreateWire(SP_Output, "wait_in");
  Op->CreateWire(SP_Output, "q", VerilogParamConst(width));
  Op->CreateWire(SP_Output, "valid_out");
  Op->CreateWire(SP_Input, "wait_out");

  Op->CreateWire(SP_Output, "bym_lockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_lockrequestvalid");
  Op->CreateWire(SP_Input, "bym_lockwaitrequest");
  Op->CreateWire(SP_Input, "bym_lockresponsevalid");
  Op->CreateWire(SP_Output, "bym_lockwaitresponse");
  Op->CreateWire(SP_Output, "bym_unlockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_unlockrequestvalid");
  Op->CreateWire(SP_Input, "bym_unlockwaitrequest");
  Op->CreateWire(SP_Input, "bym_unlockresponsevalid");
  Op->CreateWire(SP_Output, "bym_unlockwaitresponse");

  Op->CreateWire(SP_Output, "avm_address", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Output, "avm_read");
  Op->CreateWire(SP_Input, "avm_readdata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Input, "avm_readdatavalid");
  Op->CreateWire(SP_Output, "avm_write");
  Op->CreateWire(SP_Output, "avm_writedata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Output, "avm_byteenable", VerilogParamConst(DataWidthByte));
  Op->CreateWire(SP_Input, "avm_waitrequest");
  Op->CreateWire(SP_Output, "avm_waitresponse");

  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

void PlatformIPCollection::AddBinaryAtomicIP(StringRef name, unsigned width) {
  VerilogModule *Op = new VerilogModule(name);
  VerilogParameter *AddrWidth = Op->CreateParameter(PK_Param, "ADDR_WIDTH", 48);
  VerilogParameter *DataWidth = Op->CreateParameter(PK_Param, "DATA_WIDTH", 512);
  VerilogParameter *LockWidth = Op->CreateParameter(PK_Param, "LOCK_WIDTH", 4);
  VerilogParameter *DataWidthByte = Op->CreateParameter(PK_LocalParam,
                                                        "DATA_WIDTH_BYTE", 64);
  Op->CreateWire(SP_Input, "clk");
  Op->CreateWire(SP_Input, "rstn");
  Op->CreateWire(SP_Input, "addr", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Input, "a", VerilogParamConst(width));
  Op->CreateWire(SP_Input, "b", VerilogParamConst(width));
  Op->CreateWire(SP_Input, "valid_in");
  Op->CreateWire(SP_Output, "wait_in");
  Op->CreateWire(SP_Output, "q", VerilogParamConst(width));
  Op->CreateWire(SP_Output, "valid_out");
  Op->CreateWire(SP_Input, "wait_out");

  Op->CreateWire(SP_Output, "bym_lockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_lockrequestvalid");
  Op->CreateWire(SP_Input, "bym_lockwaitrequest");
  Op->CreateWire(SP_Input, "bym_lockresponsevalid");
  Op->CreateWire(SP_Output, "bym_lockwaitresponse");
  Op->CreateWire(SP_Output, "bym_unlockid", VerilogParamConst(LockWidth));
  Op->CreateWire(SP_Output, "bym_unlockrequestvalid");
  Op->CreateWire(SP_Input, "bym_unlockwaitrequest");
  Op->CreateWire(SP_Input, "bym_unlockresponsevalid");
  Op->CreateWire(SP_Output, "bym_unlockwaitresponse");

  Op->CreateWire(SP_Output, "avm_address", VerilogParamConst(AddrWidth));
  Op->CreateWire(SP_Output, "avm_read");
  Op->CreateWire(SP_Input, "avm_readdata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Input, "avm_readdatavalid");
  Op->CreateWire(SP_Output, "avm_write");
  Op->CreateWire(SP_Output, "avm_writedata", VerilogParamConst(DataWidth));
  Op->CreateWire(SP_Output, "avm_byteenable", VerilogParamConst(DataWidthByte));
  Op->CreateWire(SP_Input, "avm_waitrequest");
  Op->CreateWire(SP_Output, "avm_waitresponse");

  assert(IP.count(name) == 0);
  IP[name] = new PlatformIPCore(Op, PVars.GetVar(name, "latency"),
                                PlatformIPCore::InterpretAsLkind(PVars.GetVar(name, "type")));
}

std::string PlatformIPCollection::GetTypeCode(QualType Ty) {
  unsigned Size = (unsigned)ASTCtx.getTypeSize(Ty);
  if (Ty->isFloatingType()) {
    if (Size == 32) return "float";
    if (Size == 64) return "double";
  } else if (Ty->isSignedIntegerType()) {
    if (Size == 32) return "i32";
    if (Size == 64) return "i64";
    if (Size == 8)  return "i8";
    if (Size == 16) return "i16";
  } else {
    if (Size == 32) return "u32";
    if (Size == 64) return "u64";
    if (Size == 8)  return "u8";
    if (Size == 16) return "u16";
  }
  return "x";
}

std::string PlatformIPCollection::GetAddressSpaceCode(unsigned AS) {
  if (AS == LangAS::opencl_global) {
    return "global";
  } else if (AS == LangAS::opencl_local) {
    return "local";
  }
  return "x";
}

namespace {

inline unsigned Log2Ceil(unsigned x) {
  unsigned logx = 0;
  while (x > 1) {
    x = (x + (x % 2)) >> 1;
    logx++;
  }
  return logx;
}

} // anonymous namespace

PlatformContext::PlatformContext(const ASTContext &C, const PlatformVariables &PV,
                                 PlatformIPCollection &PI, FunctionDecl *K)
  : ASTCtx(C), PVars(PV), PIPs(PI), Kernel(K) {
  if (Kernel->hasAttr<ReqdWorkGroupSizeAttr>()) {
    ReqdWorkGroupSizeAttr *A = Kernel->getAttr<ReqdWorkGroupSizeAttr>();
    LIDWidth = Log2Ceil(A->getXDim() * A->getYDim() * A->getZDim());
  } else {
    LIDWidth = PVars.GetVar("port", "lid_width");
    if (Kernel->hasAttr<WorkGroupSizeHintAttr>()) {
      WorkGroupSizeHintAttr *A = Kernel->getAttr<WorkGroupSizeHintAttr>();
      LIDWidth = std::max(LIDWidth,
                          Log2Ceil(A->getXDim() * A->getYDim() * A->getZDim()));
    }
  }
  assert(LIDWidth > 0);
}

unsigned PlatformContext::getLmax(DFGNode *Node) {
  if (!LmaxCache.count(Node)) {
    unsigned Result = ComputeLmax(Node);
    LmaxCache[Node] = Result;
    return Result;
  } else {
    return LmaxCache[Node];
  }
}

bool PlatformContext::isVariableLatency(DFGNode *Node) {
  if (!LvarCache.count(Node)) {
    bool Result = ComputeIsVariableLatency(Node);
    LvarCache[Node] = Result;
    return Result;
  } else {
    return LvarCache[Node];
  }
}

unsigned PlatformContext::getNstall(DFGNode *Node) {
  if (!NstallCache.count(Node)) {
    unsigned Result = ComputeNstall(Node);
    NstallCache[Node] = Result;
    return Result;
  } else {
    return NstallCache[Node];
  }
}

unsigned PlatformContext::ComputeLmax(DFGNode *Node) {
  switch (Node->getClass()) {
    case DFGNode::DFGSourceClass:
    case DFGNode::DFGSinkClass: {
      return 1;
    }

    case DFGNode::DFGIntConstClass:
    case DFGNode::DFGFloatConstClass:
    case DFGNode::DFGUndefinedConstClass:
    case DFGNode::DFGPlatformConstClass:
    case DFGNode::DFGAddrOfClass:
    case DFGNode::DFGScatterClass: {
      return 0;
    }

    case DFGNode::DFGUnaryOpClass: {
      DFGUnaryOpNode *UO = static_cast<DFGUnaryOpNode*>(Node);
      switch (UO->getOpcode()) {
        case DFGUnaryOpNode::DUO_Not:
        case DFGUnaryOpNode::DUO_ReinterpretCast:
        case DFGUnaryOpNode::DUO_IntCast: {
          return 0;
        }
        case DFGUnaryOpNode::DUO_Abs: {
          if (UO->getOperand()->getType()->isUnsignedIntegerType()) {
            return 0;
          }
          break;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(UO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGBinaryOpClass: {
      DFGBinaryOpNode *BO = static_cast<DFGBinaryOpNode*>(Node);
      switch (BO->getOpcode()) {
        case DFGBinaryOpNode::DBO_And:
        case DFGBinaryOpNode::DBO_Xor:
        case DFGBinaryOpNode::DBO_Or: {
          return 0;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(BO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGTernaryOpClass: {
      DFGTernaryOpNode *TO = static_cast<DFGTernaryOpNode*>(Node);
      switch (TO->getOpcode()) {
        case DFGTernaryOpNode::DTO_Conditional: {
          return 1;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(TO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGRangeOpClass:
    case DFGNode::DFGConcatOpClass: {
      return 0;
    }

    case DFGNode::DFGVariableRangeOpClass:
    case DFGNode::DFGSubstituteOpClass: {
      return 1;
    }

    case DFGNode::DFGLoadClass: {
      DFGLoadNode *Load = static_cast<DFGLoadNode*>(Node);
      if (Load->getAddressSpace() == LangAS::opencl_global) {
        return PVars.GetVar("global_load", "latency");
      } else if (Load->getAddressSpace() == LangAS::opencl_local) {
        return PVars.GetVar("local_load", "latency");
      } else {
        llvm_unreachable("not implemented yet");
        return 0;
      }
    }

    case DFGNode::DFGStoreClass: {
      DFGStoreNode *Store = static_cast<DFGStoreNode*>(Node);
      if (Store->getAddressSpace() == LangAS::opencl_global) {
        return PVars.GetVar("global_store", "latency");
      } else if (Store->getAddressSpace() == LangAS::opencl_local) {
        return PVars.GetVar("local_store", "latency");
      } else {
        llvm_unreachable("not implemented yet");
        return 0;
      }
    }

    case DFGNode::DFGNullaryAtomicClass: {
      DFGNullaryAtomicNode *NA = static_cast<DFGNullaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(NA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGUnaryAtomicClass: {
      DFGUnaryAtomicNode *UA = static_cast<DFGUnaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGBinaryAtomicClass: {
      DFGBinaryAtomicNode *UA = static_cast<DFGBinaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGFunctionCallClass: {
      llvm_unreachable("not supported");
      return 0;
    }

    case DFGNode::DFGShiftRegisterClass: {
      DFGShiftRegisterNode *SR = static_cast<DFGShiftRegisterNode*>(Node);
      return SR->getSize();
    }

    case DFGNode::DFGQueueClass: {
      DFGQueueNode *Q = static_cast<DFGQueueNode*>(Node);
      return Q->getSize();
    }

    case DFGNode::DFGBarrierClass: {
      return (1 << LIDWidth) * 2;
    }

    case DFGNode::DFGCompoundClass: {
      DFGCompoundNode *Compound = static_cast<DFGCompoundNode*>(Node);
      unsigned Lmax = 0;
      llvm::DenseMap<DFGNode*, unsigned> Distance;
      bool Updated;
      do {
        Updated = false;
        for (DFGCompoundNode::iterator I = Compound->begin(),
                                       E = Compound->end();
             I != E; ++I) {
          DFGNode *Node = *I;
          if (!Distance.count(Node)) {
            for (DFGNode::const_pred_iterator P = Node->pred_begin(),
                                              PEnd = Node->pred_end();
                 P != PEnd; ++P) {
              if (!Compound->count(*P)) {
                Distance[Node] = getLmax(Node);
                if (Distance[Node] > Lmax) {
                  Lmax = Distance[Node];
                }
                Updated = true;
                break;
              }
              if (Distance.count(*P)) {
                Distance[Node] = Distance[*P] + getLmax(Node);
                if (Distance[Node] > Lmax) {
                  Lmax = Distance[Node];
                }
                Updated = true;
                break;
              }
            }
          }
        }
      } while (Updated);
      return Lmax;
    }
  }
  return 0;
}

bool PlatformContext::ComputeIsVariableLatency(DFGNode *Node) {
  switch (Node->getClass()) {
    case DFGNode::DFGSourceClass:
    case DFGNode::DFGSinkClass: {
      return false;
    }

    case DFGNode::DFGIntConstClass:
    case DFGNode::DFGFloatConstClass:
    case DFGNode::DFGUndefinedConstClass:
    case DFGNode::DFGPlatformConstClass:
    case DFGNode::DFGAddrOfClass:
    case DFGNode::DFGScatterClass: {
      return false;
    }

    case DFGNode::DFGUnaryOpClass: {
      DFGUnaryOpNode *UO = static_cast<DFGUnaryOpNode*>(Node);
      switch (UO->getOpcode()) {
        case DFGUnaryOpNode::DUO_Not:
        case DFGUnaryOpNode::DUO_ReinterpretCast:
        case DFGUnaryOpNode::DUO_IntCast: {
          return false;
        }
        case DFGUnaryOpNode::DUO_Abs: {
          if (UO->getOperand()->getType()->isUnsignedIntegerType()) {
            return false;
          }
          break;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(UO);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGBinaryOpClass: {
      DFGBinaryOpNode *BO = static_cast<DFGBinaryOpNode*>(Node);
      switch (BO->getOpcode()) {
        case DFGBinaryOpNode::DBO_And:
        case DFGBinaryOpNode::DBO_Xor:
        case DFGBinaryOpNode::DBO_Or: {
          return false;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(BO);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGTernaryOpClass: {
      DFGTernaryOpNode *TO = static_cast<DFGTernaryOpNode*>(Node);
      switch (TO->getOpcode()) {
        case DFGTernaryOpNode::DTO_Conditional: {
          return false;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(TO);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGRangeOpClass:
    case DFGNode::DFGVariableRangeOpClass:
    case DFGNode::DFGConcatOpClass:
    case DFGNode::DFGSubstituteOpClass: {
      return false;
    }

    case DFGNode::DFGLoadClass: {
      DFGLoadNode *Load = static_cast<DFGLoadNode*>(Node);
      if (Load->getAddressSpace() == LangAS::opencl_global) {
        return PlatformIPCore::IsVariableLatency(
            PlatformIPCore::InterpretAsLkind(PVars.GetVar("global_load", "type")));
      } else if (Load->getAddressSpace() == LangAS::opencl_local) {
        return PlatformIPCore::IsVariableLatency(
            PlatformIPCore::InterpretAsLkind(PVars.GetVar("local_load", "type")));
      } else {
        llvm_unreachable("not implemented yet");
        return 0;
      }
    }

    case DFGNode::DFGStoreClass: {
      DFGStoreNode *Store = static_cast<DFGStoreNode*>(Node);
      if (Store->getAddressSpace() == LangAS::opencl_global) {
        return PlatformIPCore::IsVariableLatency(
            PlatformIPCore::InterpretAsLkind(PVars.GetVar("global_store", "type")));
      } else if (Store->getAddressSpace() == LangAS::opencl_local) {
        return PlatformIPCore::IsVariableLatency(
            PlatformIPCore::InterpretAsLkind(PVars.GetVar("local_store", "type")));
      } else {
        llvm_unreachable("not implemented yet");
        return 0;
      }
    }

    case DFGNode::DFGNullaryAtomicClass: {
      DFGNullaryAtomicNode *NA = static_cast<DFGNullaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(NA);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGUnaryAtomicClass: {
      DFGUnaryAtomicNode *UA = static_cast<DFGUnaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGBinaryAtomicClass: {
      DFGBinaryAtomicNode *UA = static_cast<DFGBinaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->isVariableLatency();
    }

    case DFGNode::DFGFunctionCallClass: {
      llvm_unreachable("not supported");
      return false;
    }

    case DFGNode::DFGShiftRegisterClass: {
      return false;
    }

    case DFGNode::DFGQueueClass: {
      return true;
    }

    case DFGNode::DFGBarrierClass: {
      return true;
    }

    case DFGNode::DFGCompoundClass: {
      return false;
    }
  }
  return false;
}

unsigned PlatformContext::ComputeNstall(DFGNode *Node) {
  switch (Node->getClass()) {
    case DFGNode::DFGUnaryOpClass: {
      DFGUnaryOpNode *UO = static_cast<DFGUnaryOpNode*>(Node);
      switch (UO->getOpcode()) {
        case DFGUnaryOpNode::DUO_Not:
        case DFGUnaryOpNode::DUO_ReinterpretCast:
        case DFGUnaryOpNode::DUO_IntCast: {
          return 1;
        }
        case DFGUnaryOpNode::DUO_Abs: {
          if (UO->getOperand()->getType()->isUnsignedIntegerType()) {
            return 1;
          }
          break;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(UO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGBinaryOpClass: {
      DFGBinaryOpNode *BO = static_cast<DFGBinaryOpNode*>(Node);
      switch (BO->getOpcode()) {
        case DFGBinaryOpNode::DBO_And:
        case DFGBinaryOpNode::DBO_Xor:
        case DFGBinaryOpNode::DBO_Or: {
          return 1;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(BO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGTernaryOpClass: {
      DFGTernaryOpNode *TO = static_cast<DFGTernaryOpNode*>(Node);
      switch (TO->getOpcode()) {
        case DFGTernaryOpNode::DTO_Conditional: {
          return 2;
        }
        default: break;
      }
      PlatformIPCore *Impl = getIP(TO);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGNullaryAtomicClass: {
      DFGNullaryAtomicNode *NA = static_cast<DFGNullaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(NA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGUnaryAtomicClass: {
      DFGUnaryAtomicNode *UA = static_cast<DFGUnaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    case DFGNode::DFGBinaryAtomicClass: {
      DFGBinaryAtomicNode *UA = static_cast<DFGBinaryAtomicNode*>(Node);
      PlatformIPCore *Impl = getIP(UA);
      assert(Impl != NULL);
      return Impl->getLmax();
    }

    default: {
      return getLmax(Node) + 1;
    }
  }
  return 0;
}

} // namespace snu

} // namespace clang
