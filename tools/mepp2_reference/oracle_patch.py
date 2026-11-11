#!/usr/bin/env python3
"""给 MEPP2 的 Compression_Valence_Component.inl 生成「影子副本 + env 门控 dump」。

reference/MEPP2 是**只读**的（硬约束）：本脚本只读原件、把补丁写进输出路径。

dump 点选在 `Simplification()` 末尾 —— 必须在 `Compression()` **之前**，
因为后者会把 `Connectivity` 取走并清空（踩过：dump 放 `Main_Function` 里
得到的是空流，且当时看不出是空的）。

另一个踩过的点：`Compression_Valence_Component.h` 用**引号**包含 `.inl`
（`#include "Compression_Valence_Component.inl"`），引号包含优先搜索
**包含者所在目录**，所以只影子化 `.inl` 是无效的 —— `.h` 也必须一起影子化。

用法: oracle_patch.py <原件 inl> <输出 inl>
"""
import sys

ANCHOR = """  } while((Current_Number != Last_Number));

  compute_normals(_pMesh, _pm);
}"""

DUMP = """  } while((Current_Number != Last_Number));

  compute_normals(_pMesh, _pm);

  // ---- P13 ORACLE DUMP (shadow copy only; reference/MEPP2 保持只读) ----
  // Connectivity 是 push_front 累积 ⇒ 反向迭代 = 流顺序。
  // 未设 env 时一行不做；已实测与未打补丁构建的 .p3d **逐字节相同**。
  if(const char *dump_env = std::getenv("PMSF_MEPP2_ORACLE_DUMP"))
  {
    FILE *fp = std::fopen(dump_env, "w");
    if(fp)
    {
      unsigned ncomp = (unsigned)this->Connectivity.size();
      std::fprintf(fp, "# PMSF MEPP2 oracle: connectivity symbol stream\\n");
      std::fprintf(fp, "# components %u\\n", ncomp);
      for(unsigned c = 0; c < ncomp; ++c)
      {
        std::fprintf(fp, "# component %u symbols %zu layers %zu\\n",
                     c, this->Connectivity[c].size(),
                     this->NumberSymbol[c].size());
        {
          // NumberSymbol 也是 push_front ⇒ 反向迭代才是执行顺序（layer 0 = 第一次调用）。
          // 注意它是 std::list（无 operator[]），用反向迭代器 + 序号。
          {
            unsigned li = 0;
            auto vit2 = this->NumberVertices[c].rbegin();
            for(auto sit2 = this->NumberSymbol[c].rbegin();
                sit2 != this->NumberSymbol[c].rend(); ++sit2, ++li)
            {
              const int nvv = (vit2 != this->NumberVertices[c].rend()) ? *vit2 : -1;
              std::fprintf(fp, "# layer %u symbols %d vertices %d\\n", li, *sit2, nvv);
              if(vit2 != this->NumberVertices[c].rend()) ++vit2;
            }
          }
        }
        // ---- 流顺序重建 ----
        // Connectivity 经 push_front 累积，每层 drain 后该层**内部**是执行顺序，
        // 但**层与层**之间是反的（后一层整体在更前面）。故：
        //   forward(Connectivity) = [block_M][block_{M-1}]...[block_1]
        // 真正的流顺序 = block_1, block_2, ..., block_M。
        // 先前只用 rbegin()（层序对但层内反）⇒ fixture 错。这里按层切分后重排。
        {
          std::vector<int> all(this->Connectivity[c].begin(),
                               this->Connectivity[c].end());
          std::vector<std::size_t> ns;   // 每层符号数，按**执行顺序**
          for(auto it = this->NumberSymbol[c].rbegin();
              it != this->NumberSymbol[c].rend(); ++it)
            ns.push_back((std::size_t)*it);

          std::vector<std::pair<std::size_t, std::size_t>> blocks;  // (off, len)
          std::size_t off = 0;
          for(std::size_t i = ns.size(); i-- > 0; )   // 先 block_M，再 …，最后 block_1
          {
            blocks.push_back({off, ns[i]});
            off += ns[i];
          }
          std::fprintf(fp, "symbols");
          for(auto bit = blocks.rbegin(); bit != blocks.rend(); ++bit)   // block_1 先
            for(std::size_t k = 0; k < bit->second; ++k)
              std::fprintf(fp, " %d", all[bit->first + k]);
          std::fprintf(fp, "\\n");
        }
      }
      std::fclose(fp);
    }
  }
}"""

# 第二个补丁：导出**种子边**。用途见 P13_1_ORACLE_BASELINE_2026-09-22.md
# 「第二号移植风险」：种子取决于库的表示顺序，必须实测 CGAL 与 MCGAL 是否一致；
# 不一致时迁移侧直接采用本条种子（种子只有一条，作为显式元数据传递是正当的）。
SEED_ANCHOR = "  halfedge_descriptor First_halfedge = *hi;"
SEED_DUMP = SEED_ANCHOR + """
  if(const char *seed_env = std::getenv("PMSF_MEPP2_ORACLE_SEED"))
  {
    FILE *sf = std::fopen(seed_env, "w");
    if(sf)
    {
      Point3d s0 = get(*_pm, target(First_halfedge, _pMesh));
      Point3d s1 = get(*_pm, target(opposite(First_halfedge, _pMesh), _pMesh));
      std::fprintf(sf, "seed_a %.17g %.17g %.17g\\n", s0[0], s0[1], s0[2]);
      std::fprintf(sf, "seed_b %.17g %.17g %.17g\\n", s1[0], s1[1], s1[2]);
      std::fclose(sf);
    }
  }"""

# 第三个补丁：per-symbol 原因 trace。用于定位"首个符号就分岔"这类问题——
# 光有符号序列只能看到"第 N 个不同"，看不到"为什么"，而 oracle 的每一个 emit
# 都带着 val/type/原因，直接对得上就能指出是哪条分支判错了。
TRACE_DELETE_ANCHOR = """        // Enter symbol 'VALENCE CODE' into the list of symbols
        this->InterConnectivity.push_front(valence - 3);"""
TRACE_DELETE = """        if(const char *tr = std::getenv("PMSF_MEPP2_ORACLE_TRACE"))
        { (void)tr; std::fprintf(stderr, "ORACLE_TRACE sym=DELETE val=%d type=%d\\n", valence, type); }
        // Enter symbol 'VALENCE CODE' into the list of symbols
        this->InterConnectivity.push_front(valence - 3);"""

TRACE_NULL_ANCHOR = """        // Enter symbol 'NULL PATCH' into the list of symbols
        this->InterConnectivity.push_front(4);"""
TRACE_NULL = """        if(const char *tr = std::getenv("PMSF_MEPP2_ORACLE_TRACE"))
        { (void)tr; std::fprintf(stderr, "ORACLE_TRACE sym=NULL val=%d type=%d manifold=%d\\n", valence, type, (int)Manifold_condition); }
        // Enter symbol 'NULL PATCH' into the list of symbols
        this->InterConnectivity.push_front(4);"""

src_path, dst_path = sys.argv[1], sys.argv[2]
s = open(src_path).read()

# **每一个锚点都必须断言命中** —— 先前这里用裸 s.replace() 且未断言，
# 导致"补丁没打上但脚本返回成功"，白跑了一轮。别再犯。
PATCHES = [("stream", ANCHOR, DUMP, 1),
           ("seed", SEED_ANCHOR, SEED_DUMP, 1),
           ("trace_delete", TRACE_DELETE_ANCHOR, TRACE_DELETE, 1),
           ("trace_null", TRACE_NULL_ANCHOR, TRACE_NULL, 1)]
for name, anchor, _repl, _cnt in PATCHES:
    if anchor not in s:
        sys.exit("oracle_patch: anchor '%s' NOT FOUND in %s (reference changed?)" % (name, src_path))
for _name, anchor, repl, cnt in PATCHES:
    s = s.replace(anchor, repl, cnt)
open(dst_path, "w").write(s)
print("oracle_patch: %s -> %s (%d patches applied)" % (src_path, dst_path, len(PATCHES)))
