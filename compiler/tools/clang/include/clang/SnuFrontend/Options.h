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

#ifndef LLVM_CLANG_SNU_FRONTEND_OPTIONS_H
#define LLVM_CLANG_SNU_FRONTEND_OPTIONS_H

#include "llvm/Option/ArgList.h"
#include "llvm/Option/OptTable.h"
#include "llvm/Option/Option.h"
#include <string>

namespace clang {

namespace snu {

class SnuCLOptions {
public:
  std::string Prefix;
  std::string TargetFile;

  unsigned NumDatapathInstances;

  enum MemorySubsystemKind {
    MSK_BYPASS,
    MSK_CACHE
  };
  MemorySubsystemKind MemorySubsystem;

  bool LoadOrdering;
  bool PointerRemoval;

  bool VerboseCFG;
  bool VerboseAlias;
  bool VerboseOpt;

  SnuCLOptions();

  bool isVerbose() const {
    return VerboseCFG || VerboseAlias || VerboseOpt;
  }
};

namespace options {

#define SNUCL_OPTIONS(PREFIX) \
  OPTION(PREFIX, "<input>", INPUT, Input, INVALID, INVALID, 0, 0, 0, 0, 0) \
  OPTION(PREFIX, "<unknown>", UNKNOWN, Unknown, INVALID, INVALID, 0, 0, 0, 0, 0) \
  OPTION(PREFIX, "disable-pointer-removal", disable_pointer_removal, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Disable pointer removal", 0) \
  OPTION(PREFIX, "enable-load-ordering", enable_load_ordering, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Force ordering between memory loads", 0) \
  OPTION(PREFIX, "enable-pointer-removal", enable_pointer_removal, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Enable pointer removal", 0) \
  OPTION(PREFIX, "mem-bypass", mem_bypass, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Does not synthesize on-chip caches", 0) \
  OPTION(PREFIX, "mem-cache", mem_cache, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Synthesize a general-purpose on-chip cache for each buffer", 0) \
  OPTION(PREFIX, "num-datapath-instances=", num_datapath_instances_EQ, Joined, INVALID, num_datapath_instances, 0, SnuCLOption, 0, \
         "Specify the number of datapath instances", 0) \
  OPTION(PREFIX, "num-datapath-instances", num_datapath_instances, Separate, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Specify the number of datapath instances", 0) \
  OPTION(PREFIX, "prefix=", prefix_EQ, Joined, INVALID, prefix, 0, SnuCLOption, 0, \
         "Specify the output directory", 0) \
  OPTION(PREFIX, "prefix", prefix, Separate, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Specify the output directory", 0) \
  OPTION(PREFIX, "target=", target_EQ, Joined, INVALID, target, 0, SnuCLOption, 0, \
         "Specify the target.conf file (default: target.conf)", 0) \
  OPTION(PREFIX, "target", target, Separate, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Specify the target.conf file (default: target.conf)", 0) \
  OPTION(PREFIX, "valias", valias, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Print the result of the alias analysis (Steensgaard's algorithm)", 0) \
  OPTION(PREFIX, "vcfg", vcfg, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Print the CFG of every kernel", 0) \
  OPTION(PREFIX, "vopt", vopt, Flag, INVALID, INVALID, 0, SnuCLOption, 0, \
         "Print the optimization process", 0) \


// Flags specifically for SnuCL options. Must not overlap with
// llvm::opt::DriverFlag and clang::driver::options::ClangFlags
enum Flags {
  SnuCLOption = (1 << 15)
};

enum ID {
  OPT_BEGIN = 10000,
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM, \
               HELPTEXT, METAVAR) OPT_##ID,
  SNUCL_OPTIONS(0)
#undef OPTION
  OPT_END
};

llvm::opt::OptTable *createSnuCLOptTable();
bool ParseSnuCLArgs(SnuCLOptions &Opts, llvm::opt::ArgList &Args);

} // namespace options

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_FRONTEND_OPTIONS_H

