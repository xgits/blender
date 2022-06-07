
/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_uv_islands.hh"

#include <optional>

namespace blender::bke::uv_islands {

/* -------------------------------------------------------------------- */
/** \name UVIsland
 * \{ */

void UVIsland::extract_borders()
{
  /* Lookup all borders of the island. */
  Vector<UVBorderEdge> edges;
  for (int64_t prim_index = 0; prim_index < uv_primitives.size(); prim_index++) {
    UVPrimitive &prim = uv_primitives[prim_index];
    for (UVEdge *edge : prim.edges) {
      if (edge->is_border_edge()) {
        edges.append(UVBorderEdge(edge, &prim));
      }
    }
  }

  while (true) {
    std::optional<UVBorder> border = UVBorder::extract_from_edges(edges);
    if (!border.has_value()) {
      break;
    }
    borders.append(*border);
  }
}

static std::optional<UVBorderCorner> sharpest_border_corner(UVBorder &border, float *r_angle)
{
  *r_angle = std::numeric_limits<float>::max();
  std::optional<UVBorderCorner> result;
  for (UVBorderEdge &edge : border.edges) {
    if (edge.flags.extendable == false) {
      continue;
    }
    float new_radius = border.outside_angle(edge);
    if (new_radius < *r_angle) {
      *r_angle = new_radius;
      result = UVBorderCorner(&border.edges[edge.prev_index], &edge);
    }
  }
  return result;
}

static std::optional<UVBorderCorner> sharpest_border_corner(UVIsland &island)
{
  std::optional<UVBorderCorner> result;
  float sharpest_angle = std::numeric_limits<float>::max();
  for (UVBorder &border : island.borders) {
    float new_radius;
    std::optional<UVBorderCorner> new_result = sharpest_border_corner(border, &new_radius);
    if (new_radius < sharpest_angle) {
      sharpest_angle = new_radius;
      result = new_result;
    }
  }
  return result;
}

struct FanSegment {
  MeshPrimitive *primitive;
  /* UVs order are already applied. So uvs[0] mathes primitive->vertices[vert_order[0]]/ */
  float2 uvs[3];
  int vert_order[3];

  struct {
    bool found : 1;
    bool should_be_added : 1;
  } flags;

  FanSegment(MeshPrimitive *primitive, MeshVertex *vertex) : primitive(primitive)
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
  Vector<FanSegment> segments;

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
          segments.append(FanSegment(other, &vertex));
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

  void init_uv_coordinates(UVVertex &uv_vertex, const UVIsland &island)
  {
    for (FanSegment &segment : segments) {
      int2 test_edge = int2(segment.primitive->vertices[segment.vert_order[0]].vertex->v,
                            segment.primitive->vertices[segment.vert_order[1]].vertex->v);
      for (const UVPrimitive &uv_primitive : island.uv_primitives) {
        for (UVEdge *edge : uv_primitive.edges) {
          int2 o(edge->vertices[0]->vertex->v, edge->vertices[1]->vertex->v);
          if ((test_edge.x == o.x && test_edge.y == o.y) ||
              (test_edge.x == o.y && test_edge.y == o.x)) {
            segment.uvs[0] = uv_vertex.uv;
            for (int i = 0; i < 2; i++) {
              if (edge->vertices[i]->uv == uv_vertex.uv) {
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
  for (const FanSegment &segment : fan.segments) {
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

static void extend_at_vert(UVIsland &island, UVBorderCorner &corner, const MeshData &mesh_data)
{
  BLI_assert(corner.first->get_uv_vertex(1) == corner.second->get_uv_vertex(0));
  UVVertex *uv_vertex = corner.second->get_uv_vertex(0);
  Fan fan(*(uv_vertex->vertex));
  print(fan);
  fan.init_uv_coordinates(*uv_vertex, island);
  print(fan);

  for (FanSegment &segment : fan.segments) {
    segment.flags.found = false;
    MeshVertex *v0 = segment.primitive->vertices[segment.vert_order[0]].vertex;
    MeshVertex *v1 = segment.primitive->vertices[segment.vert_order[1]].vertex;
    for (UVEdge *edge : uv_vertex->uv_edges) {
      if ((edge->vertices[0]->vertex == v0 && edge->vertices[1]->vertex == v1) ||
          (edge->vertices[0]->vertex == v1 && edge->vertices[1]->vertex == v0)) {
        segment.flags.found = true;
        break;
      }
    }
  }
  print(fan);

  // tag them as being 'not fixed in uv space'. count them and determine a position in uv space.
  // add UV primitives for them.
  // recalc the border.
  int num_to_add = 0;
  for (FanSegment &segment : fan.segments) {
    if (!segment.flags.found) {
      num_to_add++;
    }
  }
  printf("Found %d new edges to add\n", num_to_add);

  if (num_to_add == 0) {
    float2 center_uv = corner.uv(0.5f);
    // no new triangles found. In this case we should extend the existing borders.
    UVVertex center_vertex;
    center_vertex.loop = uv_vertex->loop;
    center_vertex.uv = center_uv;
    center_vertex.uv_edges.clear();
    {
      MeshPrimitive *mesh_primitive = corner.second->uv_primitive->primitive;
      UVPrimitive prim1(mesh_primitive);
      MeshUVVert *other_uv_vert = mesh_primitive->get_other_uv_vertex(
          corner.second->edge->vertices[0]->vertex, corner.second->edge->vertices[1]->vertex);
      center_vertex.loop = other_uv_vert->loop;
      center_vertex.vertex = other_uv_vert->vertex;

      UVVertex *center_vertex_ptr = island.lookup_or_create(center_vertex);
      UVEdge edge_template;
      edge_template.vertices[0] = corner.first->get_uv_vertex(1);
      edge_template.vertices[1] = corner.first->get_uv_vertex(0);
      prim1.edges.append(island.lookup_or_create(edge_template));
      edge_template.vertices[0] = corner.first->get_uv_vertex(0);
      edge_template.vertices[1] = center_vertex_ptr;
      prim1.edges.append(island.lookup_or_create(edge_template));
      edge_template.vertices[0] = center_vertex_ptr;
      edge_template.vertices[1] = corner.first->get_uv_vertex(1);
      prim1.edges.append(island.lookup_or_create(edge_template));
      prim1.append_to_uv_edges();
      prim1.append_to_uv_vertices();
      island.uv_primitives.append(prim1);
    }
    {
      MeshPrimitive *mesh_primitive = corner.first->uv_primitive->primitive;
      UVPrimitive prim1(mesh_primitive);
      MeshUVVert *other_uv_vert = mesh_primitive->get_other_uv_vertex(
          corner.first->edge->vertices[0]->vertex, corner.first->edge->vertices[1]->vertex);
      center_vertex.loop = other_uv_vert->loop;
      center_vertex.vertex = other_uv_vert->vertex;
      /* TODO: Should be reversed. */
      UVVertex *center_vertex_ptr = island.lookup_or_create(center_vertex);
      UVEdge edge_template;
      edge_template.vertices[0] = corner.second->get_uv_vertex(1);
      edge_template.vertices[1] = corner.second->get_uv_vertex(0);
      prim1.edges.append(island.lookup_or_create(edge_template));
      edge_template.vertices[0] = corner.second->get_uv_vertex(0);
      edge_template.vertices[1] = center_vertex_ptr;
      prim1.edges.append(island.lookup_or_create(edge_template));
      edge_template.vertices[0] = center_vertex_ptr;
      edge_template.vertices[1] = corner.second->get_uv_vertex(1);
      prim1.edges.append(island.lookup_or_create(edge_template));
      prim1.append_to_uv_edges();
      prim1.append_to_uv_vertices();
      island.uv_primitives.append(prim1);
    }

#if 0
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
#endif
  }
  else {
  }
#if 0

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

  int64_t border_index = 0;
  for (UVBorder &border : borders) {
    border.update_indexes(border_index++);
  }

  int i = 4;
  while (i) {
    std::optional<UVBorderCorner> extension_corner = sharpest_border_corner(*this);
    if (!extension_corner.has_value()) {
      break;
    }

    /* When outside the mask, the uv should not be considered for extension. */
    if (!mask.is_masked(island_index, extension_corner->second->get_uv_vertex(0)->uv)) {
      extension_corner->second->flags.extendable = false;
      continue;
    }

    // TODO: extend
    extend_at_vert(*this, *extension_corner, mesh_data);

    /* Mark that the vert is extended. Unable to extend twice. */
    extension_corner->second->flags.extendable = false;
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

std::optional<UVBorder> UVBorder::extract_from_edges(Vector<UVBorderEdge> &edges)
{
  /* Find a part of the border that haven't been extracted yet. */
  UVBorderEdge *starting_border_edge = nullptr;
  for (UVBorderEdge &edge : edges) {
    if (edge.tag == false) {
      starting_border_edge = &edge;
      break;
    }
  }
  if (starting_border_edge == nullptr) {
    return std::nullopt;
  }
  UVBorder border;
  border.edges.append(*starting_border_edge);
  starting_border_edge->tag = true;

  float2 first_uv = starting_border_edge->get_uv_vertex(0)->uv;
  float2 current_uv = starting_border_edge->get_uv_vertex(1)->uv;
  while (current_uv != first_uv) {
    for (UVBorderEdge &border_edge : edges) {
      if (border_edge.tag == true) {
        continue;
      }
      int i;
      for (i = 0; i < 2; i++) {
        if (border_edge.edge->vertices[i]->uv == current_uv) {
          border_edge.reverse_order = i == 1;
          border_edge.tag = true;
          current_uv = border_edge.get_uv_vertex(1)->uv;
          border.edges.append(border_edge);
          break;
        }
      }
      if (i != 2) {
        break;
      }
    }
  }
  return border;
}

void UVBorder::flip_order()
{
  for (UVBorderEdge &edge : edges) {
    edge.reverse_order = !edge.reverse_order;
  }
}

float UVBorder::outside_angle(const UVBorderEdge &edge) const
{
  const UVBorderEdge &prev = edges[edge.prev_index];
  // TODO: need detection if the result is inside or outside.
  // return angle_v2v2v2(prev.uv, vert.uv, next.uv);
  return M_PI - angle_signed_v2v2(prev.get_uv_vertex(1)->uv - prev.get_uv_vertex(0)->uv,
                                  edge.get_uv_vertex(1)->uv - edge.get_uv_vertex(0)->uv);
}

void UVBorder::update_indexes(uint64_t border_index)
{
  for (int64_t i = 0; i < edges.size(); i++) {
    int64_t prev = (i - 1 + edges.size()) % edges.size();
    int64_t next = (i + 1) % edges.size();
    edges[i].prev_index = prev;
    edges[i].index = i;
    edges[i].next_index = next;
    edges[i].border_index = border_index;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name UVPrimitive
 * \{ */

UVBorder UVPrimitive::extract_border() const
{
  Vector<UVBorderEdge> border_edges;
  for (UVEdge *edge : edges) {
    /* TODO remove const cast. only needed for debugging atm. */
    UVBorderEdge border_edge(edge, const_cast<UVPrimitive *>(this));
    border_edges.append(border_edge);
  }
  return *UVBorder::extract_from_edges(border_edges);
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
  ss << "  <g fill=\"none\">\n";

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
  const UVBorder border = primitive.extract_border();
  ss << "       <polygon points=\"";
  for (const UVBorderEdge &edge : border.edges) {
    ss << " ";
    svg_coords(ss, edge.get_uv_vertex(0)->uv);
  }
  ss << "\"/>\n";

  float2 center(0.0, 0.0);
  for (const UVBorderEdge &edge : border.edges) {
    center += edge.get_uv_vertex(0)->uv;
  }
  center /= 3;

  ss << "<text x=\"" << center.x * 1024 << "\"";
  ss << " y=\"" << center.y * 1024 << "\">";
  ss << primitive.primitive->index;

  ss << "</text>\n";
  for (const UVBorderEdge &edge : border.edges) {
    float2 co = (center + edge.get_uv_vertex(0)->uv) / 2.0;
    ss << "<text x=\"" << svg_x(co) << "\"";
    ss << " y=\"" << svg_y(co) << "\">";
    ss << edge.get_uv_vertex(0)->vertex->v;
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
  for (const UVBorderEdge &edge : border.edges) {
    float2 v1 = edge.get_uv_vertex(0)->uv;
    float2 v2 = edge.get_uv_vertex(1)->uv;
    ss << "       <line x1=\"" << svg_x(v1) << "\" y1=\"" << svg_y(v1) << "\" x2=\"" << svg_x(v2)
       << "\" y2=\"" << svg_y(v2) << "\"/>\n";
  }
  ss << " </g>\n";

  ss << " <g fill=\"red\">\n";
  for (const UVBorderEdge &edge : border.edges) {
    float2 v1 = edge.get_uv_vertex(0)->uv;
    ss << "       <text x=\"" << svg_x(v1) << "\" y=\"" << svg_y(v1) << "\">"
       << (border.outside_angle(edge) / M_PI * 180) << "</text>\n";
  }
  ss << " </g>\n";

  ss << "</g>\n";
}

/** \} */

}  // namespace blender::bke::uv_islands
