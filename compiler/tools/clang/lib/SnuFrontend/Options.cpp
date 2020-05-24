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

#include "clang/SnuFrontend/Options.h"
#include "clang/Driver/Options.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Option/OptTable.h"
#include "llvm/Option/Option.h"
#include <cstdlib>
#include <string>

using namespace clang::driver;
using namespace clang::driver::options;
using namespace llvm::opt;

namespace clang {

namespace snu {

SnuCLOptions::SnuCLOptions()
  : Prefix(""),
    TargetFile("target.conf"),
    NumDatapathInstances(1),
    MemorySubsystem(MSK_CACHE),
    LoadOrdering(false),
    PointerRemoval(true),
    VerboseCFG(false),
    VerboseAlias(false),
    VerboseOpt(false) {
}

namespace options {

namespace {

static const char *const OptPrefix[] = {"-", "--", 0};

static const OptTable::Info InfoTable[] = {
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM, \
               HELPTEXT, METAVAR)   \
  { PREFIX, NAME, HELPTEXT, METAVAR, OPT_##ID, Option::KIND##Class, PARAM, \
    FLAGS, OPT_##GROUP, OPT_##ALIAS, ALIASARGS },
  SNUCL_OPTIONS(OptPrefix)
#undef OPTION
};

class SnuCLOptTable : public OptTable {
public:
  SnuCLOptTable()
    : OptTable(InfoTable, llvm::array_lengthof(InfoTable)) {}
};

} // anonymous namespace

OptTable *createSnuCLOptTable() {
  return new SnuCLOptTable();
}

bool ParseSnuCLArgs(SnuCLOptions &Opts, ArgList &Args) {
  if (Args.hasArg(OPT_prefix)) {
    Opts.Prefix = Args.getLastArgValue(OPT_prefix);
    if (!Opts.Prefix.empty() && *Opts.Prefix.rbegin() != '/') {
      Opts.Prefix.push_back('/');
    }
  }
  if (Args.hasArg(OPT_target)) {
    Opts.TargetFile = Args.getLastArgValue(OPT_target);
  }
  if (Args.hasArg(OPT_num_datapath_instances)) {
    Opts.NumDatapathInstances = atoi(Args.getLastArgValue(OPT_num_datapath_instances).data());
    assert(Opts.NumDatapathInstances >= 1);
  }
  if (Args.hasArg(OPT_mem_cache)) {
    Opts.MemorySubsystem = SnuCLOptions::MSK_CACHE;
  } else if (Args.hasArg(OPT_mem_bypass)) {
    Opts.MemorySubsystem = SnuCLOptions::MSK_BYPASS;
  }
  if (Args.hasArg(OPT_enable_load_ordering)) {
    Opts.LoadOrdering = true;
  }
  if (Args.hasArg(OPT_enable_pointer_removal)) {
    Opts.PointerRemoval = true;
  } else if (Args.hasArg(OPT_disable_pointer_removal)) {
    Opts.PointerRemoval = false;
  } else {
    Opts.PointerRemoval = true;
  }
  if (Args.hasArg(OPT_vcfg)) {
    Opts.VerboseCFG = true;
  }
  if (Args.hasArg(OPT_valias)) {
    Opts.VerboseAlias = true;
  }
  if (Args.hasArg(OPT_vopt)) {
    Opts.VerboseOpt = true;
  }
  return true;
}

} // namespace options

} // namespace snu

} // namespace clang
