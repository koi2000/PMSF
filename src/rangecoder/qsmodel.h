#ifndef QSMODEL_H
#define QSMODEL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
  qsmodel.h     headerfile for quasistatic probability model

  (c) Michael Schindler
  1997, 1998, 2000
  http://www.compressconsult.com/
  michael@compressconsult.com

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.  It may be that this
  program violates local patents in your country, however it is
  belived (NO WARRANTY!) to be patent-free here in Austria.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston,
  MA 02111-1307, USA.

  Qsmodel is a quasistatic probability model that periodically
  (at chooseable intervals) updates probabilities of symbols;
  it also allows to initialize probabilities. Updating is done more
  frequent in the beginning, so it adapts very fast even without
  initialisation.

  it provides function for creation, deletion, query for probabilities
  and symbols and model updating.

  for usage see example.c
*/

#include "port.h"

typedef struct {
    int n,             /* number of symbols */
        left,          /* symbols to next rescale */
        nextleft,      /* symbols with other increment */
        rescale,       /* intervals between rescales */
        targetrescale, /* should be interval between rescales */
        incr,          /* increment per update */
        searchshift;   /* shift for lt_freq before using as index */
    uint4 *cf,         /* array of cumulative frequencies */
        *newf,         /* array for collecting ststistics */
        *search;       /* structure for searching on decompression */
} qsmodel;

/* initialisation of qsmodel                           */
/* m   qsmodel to be initialized                       */
/* n   number of symbols in that model                 */
/* lg_totf  base2 log of total frequency count         */
/* rescale  desired rescaling interval, should be < 1<<(lg_totf+1) */
/* init  array of int's to be used for initialisation (NULL ok) */
/* compress  set to 1 on compression, 0 on decompression */
void initqsmodel( qsmodel *m, int n, int lg_totf, int rescale,
   int *init, int compress );

/* reinitialisation of qsmodel                         */
/* m   qsmodel to be initialized                       */
/* init  array of int's to be used for initialisation (NULL ok) */
void resetqsmodel( qsmodel *m, int *init);


/* deletion of qsmodel m                               */
void deleteqsmodel( qsmodel *m );


/* retrieval of estimated frequencies for a symbol     */
/* m   qsmodel to be questioned                        */
/* sym  symbol for which data is desired; must be <n   */
/* sy_f frequency of that symbol                       */
/* lt_f frequency of all smaller symbols together      */
/* the total frequency is 1<<lg_totf                   */
/* PHASE62 物品3：**内联**。这个函数体只有两行（两次 cf 读 + 一次减），
   但它是**跨 TU 的 C 函数** ⇒ 每次调用都有完整的调用开销。实测在
   armadillo 解压的热点谱上占 **18.79%**（`collectFromDecoded` 之后第二位）。
   定义搬到头里之后调用点就地展开，算术一字不改 ⇒ **产物逐位不变**。 */
void dorescale( qsmodel *m );

static inline void qsgetfreq( qsmodel *m, int sym, int *sy_f, int *lt_f )
{   *sy_f = m->cf[sym+1] - (*lt_f = m->cf[sym]);
}


/* find out symbol for a given cumulative frequency    */
/* m   qsmodel to be questioned                        */
/* lt_f  cumulative frequency                          */
/* PHASE62 物品3：同上，内联。armadillo 解压实测 3.45%。 */
static inline int qsgetsym( qsmodel *m, int lt_f )
{   int lo, hi;
    uint4 *tmp;
    tmp = m->search+(lt_f>>m->searchshift);
    lo = *tmp;
    hi = *(tmp+1) + 1;
    while (lo+1 < hi )
    {   int mid = (lo+hi)>>1;
        if (lt_f < m->cf[mid])
            hi = mid;
        else
            lo = mid;
    }
    return lo;
}


/* update model                                        */
/* m   qsmodel to be updated                           */
/* sym  symbol that occurred (must be <n from init)    */
/* PHASE62 物品3：同上，内联。armadillo 解压实测 3.39%。 */
static inline void qsupdate( qsmodel *m, int sym )
{   if (m->left <= 0)
        dorescale(m);
    m->left--;
    m->newf[sym] += m->incr;
}

#ifdef __cplusplus
}
#endif

#endif
