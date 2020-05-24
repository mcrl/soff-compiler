add_clang_library(clangSnuPreprocess
  ASTBuilder.cpp
  ASTDuplicator.cpp
  FunctionNormalizer.cpp
  )

add_dependencies(clangSnuPreprocess
  ClangStmtNodes
  )

target_link_libraries(clangSnuPreprocess
  clangAST
  clangBasic
  )
