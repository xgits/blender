
/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_uv_islands.hh"

namespace blender::bke::uv_islands {
/* -------------------------------------------------------------------- */
/** \name UVIsland
 * \{ */

void UVIsland::extract_border()
{
  Vector<UVBorderEdge> edges;
  for (int64_t prim_index = 0; prim_index < uv_primitives.size(); prim_index++) {
    UVPrimitive &prim = uv_primitives[prim_index];
    for (UVEdge *edge : prim.edges) {
      if (edge->is_border_edge()) {
        edges.append(UVBorderEdge(edge, prim_index));
      }
    }
  }

  while (true) {
    UVBorder border;
    /* Find a part of the border that haven't been extracted yet. */
    UVBorderEdge *starting_border_edge = nullptr;
    for (UVBorderEdge &edge : edges) {
      if (edge.tag == false) {
        starting_border_edge = &edge;
        break;
      }
    }
    if (starting_border_edge == nullptr) {
      break;
    }

    starting_border_edge->tag = true;
    float2 first_uv = starting_border_edge->edge->vertices[0]->uv;
    float2 current_uv = starting_border_edge->edge->vertices[1]->uv;
    MeshVertex *current_vert = starting_border_edge->edge->vertices[1]->vertex;
    border.verts.append(UVBorderVert(first_uv, starting_border_edge->edge->vertices[0]->vertex));
    while (current_uv != first_uv) {
      for (UVBorderEdge &border_edge : edges) {
        if (border_edge.tag == true) {
          continue;
        }
        int i;
        for (i = 0; i < 2; i++) {
          if (border_edge.edge->vertices[i]->uv == current_uv) {
            border.verts.append(UVBorderVert(current_uv, current_vert));
            current_uv = border_edge.edge->vertices[1 - i]->uv;
            current_vert = border_edge.edge->vertices[1 - i]->vertex;
            border_edge.tag = true;
            break;
          }
        }
        if (i != 2) {
          break;
        }
      }
    }
    borders.append(border);
  }
}

static UVBorderVert *sharpest_border_vert(UVBorder &border, float *r_angle)
{
  *r_angle = std::numeric_limits<float>::max();
  UVBorderVert *result = nullptr;
  for (UVBorderVert &vert : border.verts) {
    if (vert.flags.extendable == false) {
      continue;
    }
    float new_radius = border.outside_angle(vert);
    if (new_radius < *r_angle) {
      *r_angle = new_radius;
      result = &vert;
    }
  }
  return result;
}

static UVBorderVert *sharpest_border_vert(UVIsland &island)
{
  UVBorderVert *result = nullptr;
  float sharpest_angle = std::numeric_limits<float>::max();
  for (UVBorder &border : island.borders) {
    float new_radius;
    UVBorderVert *new_result = sharpest_border_vert(border, &new_radius);
    if (new_radius < sharpest_angle) {
      sharpest_angle = new_radius;
      result = new_result;
    }
  }
  return result;
}

struct FanTri {
  MeshPrimitive *primitive;
  /* UVs order are already applied. So uvs[0] mathes primitive->vertices[vert_order[0]]/ */
  float2 uvs[3];
  int vert_order[3];

  struct {
    bool found : 1;
    bool should_be_added : 1;
  } flags;

  FanTri(MeshPrimitive *primitive, MeshVertex *vertex) : primitive(primitive)
  {
    flags.found = false;
    flags.should_be_added = false;

    /* Reorder so the first edge starts with the given vertex. */
    if (primitive->vertices[1].vertex == vertex) {
      vert_order[0] = 1;
      vert_order[1] = 2;
      vert_order[2] = 0;
    }
    else if (primitive->vertices[2].vertex == vertex) {
      vert_order[0] = 2;
      vert_order[1] = 0;
      vert_order[2] = 1;
    }
    else {
      BLI_assert(primitive->vertices[0].vertex == vertex);
      vert_order[0] = 0;
      vert_order[1] = 1;
      vert_order[2] = 2;
    }
  }
};

struct Fan {
  /* Blades of the fan. */
  Vector<FanTri> segments;

  Fan(MeshVertex &vertex)
  {
    MeshEdge *current_edge = vertex.edges[0];
    MeshPrimitive *stop_primitive = current_edge->primitives[0];
    MeshPrimitive *previous_primitive = stop_primitive;
    while (true) {
      bool stop = false;
      for (MeshPrimitive *other : current_edge->primitives) {
        if (stop) {
          break;
        }
        if (other == previous_primitive) {
          continue;
        }

        for (MeshEdge *edge : other->edges) {
          if (edge == current_edge || (edge->vert1 != &vertex && edge->vert2 != &vertex)) {
            continue;
          }
          segments.append(FanTri(other, &vertex));
          current_edge = edge;
          previous_primitive = other;
          stop = true;
          break;
        }
      }
      printf("%lld %lld (%p)\n", stop_primitive->index, previous_primitive->index, current_edge);
      if (stop_primitive == previous_primitive) {
        break;
      }
    }
  }

  void init_uv_coordinates(UVBorderVert &vert, const UVIsland &island)
  {
    for (FanTri &segment : segments) {
      int2 test_edge = int2(segment.primitive->vertices[segment.vert_order[0]].vertex->v,
                            segment.primitive->vertices[segment.vert_order[1]].vertex->v);
      for (const UVPrimitive &uv_primitive : island.uv_primitives) {
        for (UVEdge *edge : uv_primitive.edges) {
          int2 o(edge->vertices[0]->vertex->v, edge->vertices[1]->vertex->v);
          if ((test_edge.x == o.x && test_edge.y == o.y) ||
              (test_edge.x == o.y && test_edge.y == o.x)) {
            segment.uvs[0] = vert.uv;
            for (int i = 0; i < 2; i++) {
              if (edge->vertices[i]->uv == vert.uv) {
                segment.uvs[1] = edge->vertices[1 - i]->uv;
                break;
              }
            }
          }
        }
      }
    }

    segments.last().uvs[2] = segments.first().uvs[1];
    for (int i = 0; i < segments.size() - 1; i++) {
      segments[i].uvs[2] = segments[i + 1].uvs[1];
    }
  }
};

static void print(const Fan &fan)
{
  for (const FanTri &segment : fan.segments) {
    for (int i = 0; i < 3; i++) {
      int vert_index = segment.vert_order[i];
      printf("%lld(%f,%f) ",
             segment.primitive->vertices[vert_index].vertex->v,
             segment.uvs[i].x,
             segment.uvs[i].y);
    }
    printf(" %d\n", segment.flags.found);
  }
}

static void extend_at_vert(UVIsland &island, UVBorderVert &vert, const MeshData &mesh_data)
{
  Fan fan(*vert.vertex);
  print(fan);
  fan.init_uv_coordinates(vert, island);
  print(fan);

#if 0
  // add all verts that arent connected to the given border vert to the UVIsland.
  for (FanTri &tri : fan.tris) {
    tri.flags.found = false;
    int2 test_edge(tri.v[0], tri.v[1]);
    for (UVPrimitive &prim : island.uv_primitives) {
      for (UVEdge &edge : prim.edges) {
        if (edge.vertices[0].uv == vert.uv || edge.vertices[1].uv == vert.uv) {
          int2 o(mesh_data.mloop[edge.vertices[0].loop].v,
                 mesh_data.mloop[edge.vertices[1].loop].v);
          if ((test_edge.x == o.x && test_edge.y == o.y) ||
              (test_edge.x == o.y && test_edge.y == o.x)) {
            tri.flags.found = true;
          }
        }
      }
    }
  }
  print(fan);
  // tag them as being 'not fixed in uv space'. count them and determine a position in uv space.
  // add UV primitives for them.
  // recalc the border.
  int num_to_add = 0;
  for (FanTri &tri : fan.tris) {
    if (!tri.flags.found) {
      num_to_add++;
    }
  }
  printf("Found %d new edges to add\n", num_to_add);

  if (num_to_add > 0) {
    UVBorder &border = island.borders[vert.border_index];
    int i = 0;

    {
      UVPrimitive test(0);
      test.edges[0].vertices[0].uv = border.verts[vert.index].uv;
      test.edges[0].vertices[1].uv = border.verts[vert.next_index].uv;
      test.edges[1].vertices[0].uv = border.verts[vert.next_index].uv;
      test.edges[1].vertices[1].uv = border.verts[vert.prev_index].uv;
      test.edges[2].vertices[0].uv = border.verts[vert.prev_index].uv;
      test.edges[2].vertices[1].uv = border.verts[vert.index].uv;
      // island.primitives.append(test);
      // return;
    }
    FanTri *prev_tri = &fan.tris.last();
    for (int tri_index = 0; tri_index < fan.tris.size(); tri_index++) {
      FanTri &tri = fan.tris[tri_index];
      if (tri.flags.found) {
        prev_tri = &tri;
        continue;
      }
      float factor = float(i + 1) / float(num_to_add + 1);
      float2 new_pos;
      interp_v2_v2v2(
          new_pos, border.verts[vert.prev_index].uv, border.verts[vert.next_index].uv, factor);
      print_v2_id(new_pos);
      // TODO change length of edge.

      tri.uvs[1] = new_pos;
      prev_tri->uvs[2] = new_pos;
      tri.flags.should_be_added = true;
      prev_tri->flags.should_be_added = true;

      i++;
      prev_tri = &tri;
    }
    print(fan);

    for (FanTri &tri : fan.tris) {
      if (!tri.flags.should_be_added) {
        continue;
      }
      /*
      UVPrimitive prim(tri.prim_index);
      prim.edges[0].vertices[0].uv = tri.uvs[0];
      prim.edges[0].vertices[1].uv = tri.uvs[1];
      prim.edges[1].vertices[0].uv = tri.uvs[1];
      prim.edges[1].vertices[1].uv = tri.uvs[2];
      prim.edges[2].vertices[0].uv = tri.uvs[2];
      prim.edges[2].vertices[1].uv = tri.uvs[0];

      // prim.edges[0].adjacent_uv_primitive = prev_tri.uv_prim_index;
      // prim.edges[1].adjacent_uv_primitive = next_tri.uv_prim_index;
      island.primitives.append(prim);
      */
    }
  }
  else {
    // TODO duplicate tris or fill tri.
    // Currently we only do the duplication.
    /*
    UVBorder &border = island.borders[vert.border_index];

    float2 center_uv = (border.verts[vert.next_index].uv + border.verts[vert.prev_index].uv) /
                       2.0f;
    UVPrimitive prim1(0);
    prim1.edges[0].vertices[0].uv = border.verts[vert.index].uv;
    prim1.edges[0].vertices[1].uv = border.verts[vert.prev_index].uv;
    prim1.edges[1].vertices[0].uv = border.verts[vert.prev_index].uv;
    prim1.edges[1].vertices[1].uv = center_uv;
    prim1.edges[2].vertices[0].uv = center_uv;
    prim1.edges[2].vertices[1].uv = border.verts[vert.index].uv;
    island.uv_primitives.append(prim1);

    UVPrimitive prim2(0);
    prim2.edges[0].vertices[0].uv = border.verts[vert.index].uv;
    prim2.edges[0].vertices[1].uv = center_uv;
    prim2.edges[1].vertices[0].uv = center_uv;
    prim2.edges[1].vertices[1].uv = border.verts[vert.next_index].uv;
    prim2.edges[2].vertices[0].uv = border.verts[vert.next_index].uv;
    prim2.edges[2].vertices[1].uv = border.verts[vert.index].uv;
    island.uv_primitives.append(prim2);

    vert.uv = center_uv;
    */
  }

  // count fan-sections between border edges.
  // 0 : split in half.
#endif
}

void UVIsland::extend_border(const UVIslandsMask &mask,
                             const short island_index,
                             const MeshData &mesh_data)
{
  // Find sharpest corner that still inside the island mask and can be extended.
  // exit when no corner could be found.
#ifdef DEBUG_SVG
  int step = 0;
  std::ofstream of;
  of.open("/tmp/extend.svg");
  svg_header(of);
  svg(of, *this, step++);
#endif

  int i = 5;
  while (i) {
    int64_t border_index = 0;
    for (UVBorder &border : borders) {
      border.update_indexes(border_index++);
    }

    UVBorderVert *extension_vert = sharpest_border_vert(*this);
    if (extension_vert == nullptr) {
      break;
    }

    /* When outside the mask, the uv should not be considered for extension. */
    if (!mask.is_masked(island_index, extension_vert->uv)) {
      extension_vert->flags.extendable = false;
      continue;
    }

    // TODO: extend
    extend_at_vert(*this, *extension_vert, mesh_data);

    /* Mark that the vert is extended. Unable to extend twice. */
    extension_vert->flags.extendable = false;
    i--;
#ifdef DEBUG_SVG
    svg(of, *this, step++);
#endif
  }
#ifdef DEBUG_SVG
  svg_footer(of);
  of.close();
#endif
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name UVBorder
 * \{ */
void flip_order()
{
  // TODO implement.
}

static const UVBorderVert *previous_vert(const UVBorder &border, const UVBorderVert &vert)
{
  const UVBorderVert *result = &border.verts.last();
  for (const UVBorderVert &other : border.verts) {
    if (other.uv == vert.uv) {
      return result;
    }
    result = &other;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static const UVBorderVert *next_vert(const UVBorder &border, const UVBorderVert &vert)
{
  const UVBorderVert *result = &border.verts.first();
  // TODO: tried to use reversed_iterator, but wasn't able to solve its compiler issues ATM. will
  // look at it later on.
  for (int i = border.verts.size() - 1; i >= 0; i--) {
    const UVBorderVert &other = border.verts[i];
    if (other.uv == vert.uv) {
      return result;
    }
    result = &other;
  }
  BLI_assert_unreachable();
  return nullptr;
}

float UVBorder::outside_angle(const UVBorderVert &vert) const
{
  const UVBorderVert &prev = *previous_vert(*this, vert);
  const UVBorderVert &next = *next_vert(*this, vert);
  // TODO: need detection if the result is inside or outside.
  // return angle_v2v2v2(prev.uv, vert.uv, next.uv);
  return M_PI - angle_signed_v2v2(vert.uv - prev.uv, next.uv - vert.uv);
}

void UVBorder::update_indexes(uint64_t border_index)
{
  for (int64_t i = 0; i < verts.size(); i++) {
    int64_t prev = (i - 1 + verts.size()) % verts.size();
    int64_t next = (i + 1) % verts.size();
    verts[i].prev_index = prev;
    verts[i].index = i;
    verts[i].next_index = next;
    verts[i].border_index = border_index;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name UVIslandsMask
 * \{ */

static bool dilate_x(UVIslandsMask &islands_mask)
{
  bool changed = false;
  const Array<uint16_t> prev_mask = islands_mask.mask;
  for (int y = 0; y < islands_mask.resolution.y; y++) {
    for (int x = 0; x < islands_mask.resolution.x; x++) {
      uint64_t offset = y * islands_mask.resolution.x + x;
      if (prev_mask[offset] != 0xffff) {
        continue;
      }
      if (x != 0 && prev_mask[offset - 1] != 0xffff) {
        islands_mask.mask[offset] = prev_mask[offset - 1];
        changed = true;
      }
      else if (x < islands_mask.resolution.x - 1 && prev_mask[offset + 1] != 0xffff) {
        islands_mask.mask[offset] = prev_mask[offset + 1];
        changed = true;
      }
    }
  }
  return changed;
}

static bool dilate_y(UVIslandsMask &islands_mask)
{
  bool changed = false;
  const Array<uint16_t> prev_mask = islands_mask.mask;
  for (int y = 0; y < islands_mask.resolution.y; y++) {
    for (int x = 0; x < islands_mask.resolution.x; x++) {
      uint64_t offset = y * islands_mask.resolution.x + x;
      if (prev_mask[offset] != 0xffff) {
        continue;
      }
      if (y != 0 && prev_mask[offset - islands_mask.resolution.x] != 0xffff) {
        islands_mask.mask[offset] = prev_mask[offset - islands_mask.resolution.x];
        changed = true;
      }
      else if (y < islands_mask.resolution.y - 1 &&
               prev_mask[offset + islands_mask.resolution.x] != 0xffff) {
        islands_mask.mask[offset] = prev_mask[offset + islands_mask.resolution.x];
        changed = true;
      }
    }
  }
  return changed;
}

void UVIslandsMask::dilate()
{
#ifdef DEBUG_SVG
  std::ofstream of;
  of.open("/tmp/dilate.svg");
  svg_header(of);
  int index = 0;
#endif
  while (true) {
    bool changed = dilate_x(*this);
    changed |= dilate_y(*this);
    if (!changed) {
      break;
    }
#ifdef DEBUG_SVG
    svg(of, *this, index++);
#endif
  }
#ifdef DEBUG_SVG
  svg(of, *this, index);
  svg_footer(of);
  of.close();
#endif
}

bool UVIslandsMask::is_masked(const short island_index, const float2 uv) const
{
  float2 local_uv = uv - udim_offset;
  if (local_uv.x < 0.0f || local_uv.y < 0.0f || local_uv.x >= 1.0f || local_uv.y >= 1.0f) {
    return false;
  }
  float2 pixel_pos_f = local_uv * float2(resolution.x, resolution.y);
  ushort2 pixel_pos = ushort2(pixel_pos_f.x, pixel_pos_f.y);
  uint64_t offset = pixel_pos.y * resolution.x + pixel_pos.x;
  return mask[offset] == island_index;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name SVG export
 *
 * Debugging functions to export UV islands to SVG files.
 * \{ */
static float svg_x(const float2 &uv)
{
  return uv.x * 1024;
}

static float svg_y(const float2 &uv)
{
  return uv.y * 1024;
}

static float svg_x(const UVVertex &vertex)
{
  return svg_x(vertex.uv);
}

static float svg_y(const UVVertex &vertex)
{
  return svg_y(vertex.uv);
}

void svg_header(std::ostream &ss)
{
  ss << "<svg viewBox=\"0 0 1024 1024\" width=\"1024\" height=\"1024\" "
        "xmlns=\"http://www.w3.org/2000/svg\">\n";
}
void svg_footer(std::ostream &ss)
{
  ss << "</svg>\n";
}
void svg(std::ostream &ss, const UVEdge &edge)
{
  ss << "       <line x1=\"" << svg_x(*edge.vertices[0]) << "\" y1=\"" << svg_y(*edge.vertices[0])
     << "\" x2=\"" << svg_x(*edge.vertices[1]) << "\" y2=\"" << svg_y(*edge.vertices[1])
     << "\"/>\n";
}

void svg(std::ostream &ss, const UVIsland &island, int step)
{
  ss << "<g transform=\"translate(" << step * 1024 << " 0)\">\n";
  ss << "  <g fill=\"yellow\">\n";

  /* Inner edges */
  ss << "    <g stroke=\"grey\" stroke-width=\"1\">\n";
  for (const UVPrimitive &primitive : island.uv_primitives) {
    svg(ss, primitive);
  }
  ss << "     </g>\n";

  /* Border */
  ss << "    <g stroke=\"black\" stroke-width=\"2\">\n";
  for (const UVPrimitive &primitive : island.uv_primitives) {
    for (int i = 0; i < 3; i++) {
      const UVEdge &edge = *primitive.edges[i];
      if (!edge.is_border_edge()) {
        continue;
      }
      svg(ss, edge);
    }
  }
  ss << "     </g>\n";

  ss << "   </g>\n";

  ss << "</g>\n";
}

void svg(std::ostream &ss, const UVIslands &islands, int step)
{
  ss << "<g transform=\"translate(" << step * 1024 << " 0)\">\n";
  int island_index = 0;
  for (const UVIsland &island : islands.islands) {
    ss << "  <g fill=\"yellow\">\n";

    /* Inner edges */
    ss << "    <g stroke=\"grey\" stroke-dasharray=\"5 5\">\n";
    for (const UVPrimitive &primitive : island.uv_primitives) {
      for (int i = 0; i < 3; i++) {
        const UVEdge &edge = *primitive.edges[i];
        if (edge.is_border_edge()) {
          continue;
        }
        svg(ss, edge);
      }
    }
    ss << "     </g>\n";

    /* Border */
    ss << "    <g stroke=\"black\" stroke-width=\"2\">\n";
    for (const UVPrimitive &primitive : island.uv_primitives) {
      for (int i = 0; i < 3; i++) {
        const UVEdge &edge = *primitive.edges[i];
        if (!edge.is_border_edge()) {
          continue;
        }
        svg(ss, edge);
      }
    }
    ss << "     </g>\n";

    ss << "   </g>\n";
    island_index++;
  }

  ss << "</g>\n";
}

void svg(std::ostream &ss, const UVIslandsMask &mask, int step)
{
  ss << "<g transform=\"translate(" << step * 1024 << " 0)\">\n";
  ss << " <g fill=\"none\" stroke=\"black\">\n";

  float2 resolution = float2(mask.resolution.x, mask.resolution.y);
  for (int x = 0; x < mask.resolution.x; x++) {
    for (int y = 0; y < mask.resolution.y; y++) {
      int offset = y * mask.resolution.x + x;
      int offset2 = offset - 1;
      if (y == 0 && mask.mask[offset] == 0xffff) {
        continue;
      }
      if (x > 0 && mask.mask[offset] == mask.mask[offset2]) {
        continue;
      }
      float2 start = float2(float(x), float(y)) / resolution * float2(1024, 1024);
      float2 end = float2(float(x), float(y + 1)) / resolution * float2(1024, 1024);
      ss << "       <line x1=\"" << start.x << "\" y1=\"" << start.y << "\" x2=\"" << end.x
         << "\" y2=\"" << end.y << "\"/>\n";
    }
  }

  for (int x = 0; x < mask.resolution.x; x++) {
    for (int y = 0; y < mask.resolution.y; y++) {
      int offset = y * mask.resolution.x + x;
      int offset2 = offset - mask.resolution.x;
      if (x == 0 && mask.mask[offset] == 0xffff) {
        continue;
      }
      if (y > 0 && mask.mask[offset] == mask.mask[offset2]) {
        continue;
      }
      float2 start = float2(float(x), float(y)) / resolution * float2(1024, 1024);
      float2 end = float2(float(x + 1), float(y)) / resolution * float2(1024, 1024);
      ss << "       <line x1=\"" << start.x << "\" y1=\"" << start.y << "\" x2=\"" << end.x
         << "\" y2=\"" << end.y << "\"/>\n";
    }
  }
  ss << " </g>\n";
  ss << "</g>\n";
}

void svg_coords(std::ostream &ss, const float2 &coords)
{
  ss << coords.x * 1024 << "," << coords.y * 1024;
}

void svg(std::ostream &ss, const UVPrimitive &primitive)
{
  ss << "       <polygon points=\"";
  for (int i = 0; i < 3; i++) {
    svg_coords(ss, primitive.edges[i]->vertices[0]->uv);
    ss << " ";
  }
  ss << "\"/>\n";

  float2 center(0.0, 0.0);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      center += primitive.edges[i]->vertices[j]->uv;
    }
  }
  center /= 3 * 2;
  ss << "<text x=\"" << center.x * 1024 << "\"";
  ss << " y=\"" << center.y * 1024 << "\">";
  ss << primitive.primitive->index;
  ss << "</text>\n";
  for (int i = 0; i < 3; i++) {
    float2 co = (center + primitive.edges[i]->vertices[0]->uv) / 2.0;
    ss << "<text x=\"" << co.x * 1024 << "\"";
    ss << " y=\"" << co.y * 1024 << "\">";
    ss << primitive.edges[i]->vertices[0]->vertex->v;
    ss << "</text>\n";
  }
}

void svg(std::ostream &ss, const UVPrimitive &primitive, int step)
{
  ss << "<g transform=\"translate(" << step * 1024 << " 0)\">\n";
  ss << "  <g fill=\"red\">\n";
  svg(ss, primitive);
  ss << "  </g>";
  ss << "</g>\n";
}

void svg(std::ostream &ss, const UVBorder &border)
{
  ss << "<g>\n";

  ss << " <g stroke=\"lightgrey\">\n";
  for (const UVBorderVert &vert : border.verts) {
    const UVBorderVert &prev = *previous_vert(border, vert);
    float2 v1 = prev.uv * float2(1024, 1024);
    float2 v2 = vert.uv * float2(1024, 1024);
    ss << "       <line x1=\"" << v1.x << "\" y1=\"" << v1.y << "\" x2=\"" << v2.x << "\" y2=\""
       << v2.y << "\"/>\n";
  }
  ss << " </g>\n";

  ss << " <g fill=\"red\">\n";
  for (const UVBorderVert &vert : border.verts) {
    float2 v1 = vert.uv * float2(1024, 1024);
    ss << "       <text x=\"" << v1.x << "\" y=\"" << v1.y << "\">"
       << (border.outside_angle(vert) / M_PI * 180) << "</text>\n";
  }
  ss << " </g>\n";

  ss << "</g>\n";
}

/** \} */

}  // namespace blender::bke::uv_islands
