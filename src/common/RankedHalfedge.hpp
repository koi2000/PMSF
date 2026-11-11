/*****************************************************************************
 * Copyright (C) 2013 Adrien Maglo
 *
 * This file is part of POMAR.
 *
 * POMAR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * POMAR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with POMAR.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef RANKEDHALFEDGE_HPP
#define RANKEDHALFEDGE_HPP

#include "Halfedge.h"

struct RankedHalfedge {
    RankedHalfedge(MCGAL::Halfedge* heh) : heh(heh) {
        f_length = heh->length();
    }

    bool operator<(const RankedHalfedge rh) const {
        return rh.f_length > f_length;
    }

    MCGAL::Halfedge* heh;
    float f_length;
};

#endif  // RANKEDHALFEDGE_HPP
