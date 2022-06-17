
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
    if (!border->is_ccw()) {
      border->flip();
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
    float new_angle = border.outside_angle(edge);
    if (new_angle < *r_angle) {
      *r_angle = new_angle;
      result = UVBorderCorner(&border.edges[edge.prev_index], &edge, new_angle);
    }
  }
  return result;
}

static std::optional<UVBorderCorner> sharpest_border_corner(UVIsland &island)
{
  std::optional<UVBorderCorner> result;
  float sharpest_angle = std::numeric_limits<float>::max();
  for (UVBorder &border : island.borders) {
    float new_angle;
    std::optional<UVBorderCorner> new_result = sharpest_border_corner(border, &new_angle);
    if (new_angle < sharpest_angle) {
      sharpest_angle = new_angle;
      result = new_result;
    }
  }
  return result;
}

/** The inner edge of a fan. */
struct InnerEdge {
  MeshPrimitive *primitive;
  /* UVs order are already applied. So uvs[0] mathes primitive->vertices[vert_order[0]]/ */
  float2 uvs[3];
  int vert_order[3];

  struct {
    bool found : 1;
  } flags;

  InnerEdge(MeshPrimitive *primitive, MeshVertex *vertex) : primitive(primitive)
  {
    flags.found = false;

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

// TODO: should have InnerEdges and Primitives to reduce complexity in algorithm
struct Fan {
  /* Blades of the fan. */
  Vector<InnerEdge> inner_edges;

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
          inner_edges.append(InnerEdge(other, &vertex));
          current_edge = edge;
          previous_primitive = other;
          stop = true;
          break;
        }
      }
      if (stop == false) {
        printf("unknown how to create the fan for vert %lld\n", vertex.v);
        break;
      }
      if (stop_primitive == previous_primitive) {
        break;
      }
    }
  }

  int count_num_to_add() const
  {
    int result = 0;
    for (const InnerEdge &fan_edge : inner_edges) {
      if (!fan_edge.flags.found) {
        result++;
      }
    }
    return result;
  }

  void mark_already_added_segments(const UVVertex &uv_vertex)
  {
    for (InnerEdge &fan_edge : inner_edges) {
      fan_edge.flags.found = false;
      const MeshVertex *v0 = fan_edge.primitive->vertices[fan_edge.vert_order[0]].vertex;
      const MeshVertex *v1 = fan_edge.primitive->vertices[fan_edge.vert_order[1]].vertex;
      for (const UVEdge *edge : uv_vertex.uv_edges) {
        const MeshVertex *e0 = edge->vertices[0]->vertex;
        const MeshVertex *e1 = edge->vertices[1]->vertex;
        if ((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0)) {
          fan_edge.flags.found = true;
          break;
        }
      }
    }
  }

  void init_uv_coordinates(UVVertex &uv_vertex, const UVIsland &island)
  {
    for (InnerEdge &fan_edge : inner_edges) {
      int2 test_edge = int2(fan_edge.primitive->vertices[fan_edge.vert_order[0]].vertex->v,
                            fan_edge.primitive->vertices[fan_edge.vert_order[1]].vertex->v);
      for (const UVPrimitive &uv_primitive : island.uv_primitives) {
        for (UVEdge *edge : uv_primitive.edges) {
          int2 o(edge->vertices[0]->vertex->v, edge->vertices[1]->vertex->v);
          if ((test_edge.x == o.x && test_edge.y == o.y) ||
              (test_edge.x == o.y && test_edge.y == o.x)) {
            fan_edge.uvs[0] = uv_vertex.uv;
            for (int i = 0; i < 2; i++) {
              if (edge->vertices[i]->uv == uv_vertex.uv) {
                fan_edge.uvs[1] = edge->vertices[1 - i]->uv;
                break;
              }
            }
          }
        }
      }
    }

    inner_edges.last().uvs[2] = inner_edges.first().uvs[1];
    for (int i = 0; i < inner_edges.size() - 1; i++) {
      inner_edges[i].uvs[2] = inner_edges[i + 1].uvs[1];
    }
  }
};

static void print(const Fan &fan)
{
  for (const InnerEdge &fan_edge : fan.inner_edges) {
    for (int i = 0; i < 3; i++) {
      int vert_index = fan_edge.vert_order[i];
      printf("%lld(%f,%f) ",
             fan_edge.primitive->vertices[vert_index].vertex->v,
             fan_edge.uvs[i].x,
             fan_edge.uvs[i].y);
    }
    printf(" %d\n", fan_edge.flags.found);
  }
}

static void add_uv_primitive_shared_uv_edge(UVIsland &island,
                                            UVVertex *connected_vert_1,
                                            UVVertex *connected_vert_2,
                                            float2 uv_unconnected,
                                            MeshPrimitive *mesh_primitive)
{
  UVPrimitive prim1(mesh_primitive);

  MeshUVVert *other_vert = mesh_primitive->get_other_uv_vertex(connected_vert_1->vertex,
                                                               connected_vert_2->vertex);
  UVVertex vert_template;
  vert_template.uv = uv_unconnected;
  vert_template.vertex = other_vert->vertex;
  UVVertex *vert_ptr = island.lookup_or_create(vert_template);

  const MeshUVVert *mesh_vert_1 = &mesh_primitive->get_uv_vert(connected_vert_1->vertex);
  vert_template.uv = connected_vert_1->uv;
  vert_template.vertex = mesh_vert_1->vertex;
  UVVertex *vert_1_ptr = island.lookup_or_create(vert_template);

  const MeshUVVert *mesh_vert_2 = &mesh_primitive->get_uv_vert(connected_vert_2->vertex);
  vert_template.uv = connected_vert_2->uv;
  vert_template.vertex = mesh_vert_2->vertex;
  UVVertex *vert_2_ptr = island.lookup_or_create(vert_template);

  UVEdge edge_template;
  edge_template.vertices[0] = vert_1_ptr;
  edge_template.vertices[1] = vert_2_ptr;
  prim1.edges.append(island.lookup_or_create(edge_template));
  edge_template.vertices[0] = vert_2_ptr;
  edge_template.vertices[1] = vert_ptr;
  prim1.edges.append(island.lookup_or_create(edge_template));
  edge_template.vertices[0] = vert_ptr;
  edge_template.vertices[1] = vert_1_ptr;
  prim1.edges.append(island.lookup_or_create(edge_template));
  prim1.append_to_uv_edges();
  prim1.append_to_uv_vertices();
  island.uv_primitives.append(prim1);
  island.validate_primitive(island.uv_primitives.last());
}

static MeshPrimitive *find_fill_border(const MeshVertex &v1,
                                       const MeshVertex &v2,
                                       const MeshVertex &v3)
{
  printf("find primitive containing (%lld,%lld,%lld)\n", v1.v, v2.v, v3.v);
  for (MeshEdge *edge : v1.edges) {
    for (MeshPrimitive *primitive : edge->primitives) {
      printf("- try primitive %lld containing (%lld,%lld,%lld)\n",
             primitive->index,
             primitive->vertices[0].vertex->v,
             primitive->vertices[1].vertex->v,
             primitive->vertices[2].vertex->v);
      if (primitive->has_vertex(v1) && primitive->has_vertex(v2) && primitive->has_vertex(v3)) {
        printf("- found primitive\n");
        return primitive;
      }
    }
  }
  printf("- No primitive found\n");
  return nullptr;
}
/**
 * Find a primitive that can be used to fill give corner.
 * Will return nullptr when no primitive can be found.
 */
static MeshPrimitive *find_fill_border(UVBorderCorner &corner)
{
  if (corner.first->get_uv_vertex(1) != corner.second->get_uv_vertex(0)) {
    return nullptr;
  }
  if (corner.first->get_uv_vertex(0) == corner.second->get_uv_vertex(1)) {
    return nullptr;
  }
  UVVertex *shared_vert = corner.second->get_uv_vertex(0);
  for (MeshEdge *edge : shared_vert->vertex->edges) {
    if (corner.first->edge->has_same_vertices(*edge)) {
      for (MeshPrimitive *primitive : edge->primitives) {
        MeshVertex *other_vert = primitive->get_other_uv_vertex(edge->vert1, edge->vert2)->vertex;
        if (other_vert == corner.second->get_uv_vertex(1)->vertex) {
          return primitive;
        }
      }
    }
  }
  return nullptr;
}

static void add_uv_primitive_fill(UVIsland &island,
                                  UVVertex &uv_vertex1,
                                  UVVertex &uv_vertex2,
                                  UVVertex &uv_vertex3,
                                  MeshPrimitive &fill_primitive)
{
  UVPrimitive uv_primitive(&fill_primitive);
  UVEdge edge_template;
  edge_template.vertices[0] = &uv_vertex1;
  edge_template.vertices[1] = &uv_vertex2;
  uv_primitive.edges.append(island.lookup_or_create(edge_template));
  edge_template.vertices[0] = &uv_vertex2;
  edge_template.vertices[1] = &uv_vertex3;
  uv_primitive.edges.append(island.lookup_or_create(edge_template));
  edge_template.vertices[0] = &uv_vertex3;
  edge_template.vertices[1] = &uv_vertex1;
  uv_primitive.edges.append(island.lookup_or_create(edge_template));
  uv_primitive.append_to_uv_edges();
  uv_primitive.append_to_uv_vertices();
  island.uv_primitives.append(uv_primitive);
  // island.validate_primitive(island.uv_primitives.last());
}

static void extend_at_vert(UVIsland &island, UVBorderCorner &corner, const MeshData &mesh_data)
{
  int border_index = corner.first->border_index;
  UVBorder &border = island.borders[border_index];

  UVVertex *uv_vertex = corner.second->get_uv_vertex(0);
  Fan fan(*(uv_vertex->vertex));
  fan.init_uv_coordinates(*uv_vertex, island);
  fan.mark_already_added_segments(*uv_vertex);
  print(fan);

  // tag them as being 'not fixed in uv space'. count them and determine a position in uv space.
  // add UV primitives for them.
  // recalc the border.
  int num_to_add = fan.count_num_to_add();
  printf("Found %d new edges to add\n", num_to_add);

  if (num_to_add == 0) {
    MeshPrimitive *fill_primitive_1 = corner.second->uv_primitive->primitive;
    MeshPrimitive *fill_primitive_2 = corner.first->uv_primitive->primitive;

    MeshPrimitive *fill_primitive = find_fill_border(corner);
    // Although the fill_primitive can fill the missing segment it could lead to a squashed
    // triangle when the corner angle is near 180 degrees. In order to fix this we will
    // always add two segments both using the found fill primitive.
    if (fill_primitive) {
      fill_primitive_1 = fill_primitive;
      fill_primitive_2 = fill_primitive;
    }

    float2 center_uv = corner.uv(0.5f);
    printf(" - add new projection for {%lld}\n", fill_primitive_1->index);
    add_uv_primitive_shared_uv_edge(island,
                                    corner.first->get_uv_vertex(1),
                                    corner.first->get_uv_vertex(0),
                                    center_uv,
                                    fill_primitive_1);
    printf(" - add new projection for {%lld}\n", fill_primitive_2->index);
    add_uv_primitive_shared_uv_edge(island,
                                    corner.second->get_uv_vertex(0),
                                    corner.second->get_uv_vertex(1),
                                    center_uv,
                                    fill_primitive_2);
    /* Update border after adding the new geometry. */
    {
      UVPrimitive &new_prim = island.uv_primitives[island.uv_primitives.size() - 2];
      UVBorderEdge *border_edge = corner.first;
      border_edge->uv_primitive = &new_prim;
      border_edge->edge = border_edge->uv_primitive->get_uv_edge(
          corner.first->get_uv_vertex(0)->uv, center_uv);
      border_edge->reverse_order = border_edge->edge->vertices[0]->uv == center_uv;
    }
    {
      UVPrimitive &new_prim = island.uv_primitives[island.uv_primitives.size() - 1];
      UVBorderEdge *border_edge = corner.second;
      border_edge->uv_primitive = &new_prim;
      border_edge->edge = border_edge->uv_primitive->get_uv_edge(
          corner.second->get_uv_vertex(1)->uv, center_uv);
      border_edge->reverse_order = border_edge->edge->vertices[1]->uv == center_uv;
    }
  }
  else {
    UVEdge *current_edge = corner.first->edge;

    Vector<UVBorderEdge> new_border_edges;

    for (int i = 0; i < num_to_add; i++) {
      float2 old_uv = current_edge->get_other_uv_vertex(uv_vertex->vertex)->uv;
      MeshVertex *shared_edge_vertex =
          current_edge->get_other_uv_vertex(uv_vertex->vertex)->vertex;

      float factor = (i + 1.0f) / (num_to_add + 1.0f);
      float2 new_uv = corner.uv(factor);

      // Find an segment that contains the 'current edge'.
      for (InnerEdge &segment : fan.inner_edges) {
        if (segment.flags.found) {
          continue;
        }

        // Find primitive that shares the current edge and the segment edge.
        MeshPrimitive *fill_primitive = find_fill_border(
            *uv_vertex->vertex,
            *shared_edge_vertex,
            *segment.primitive->vertices[segment.vert_order[1]].vertex);
        if (fill_primitive == nullptr) {
          continue;
        }
        MeshVertex *other_prim_vertex =
            fill_primitive->get_other_uv_vertex(uv_vertex->vertex, shared_edge_vertex)->vertex;

        UVVertex uv_vertex_template;
        uv_vertex_template.vertex = uv_vertex->vertex;
        uv_vertex_template.uv = uv_vertex->uv;
        UVVertex *vertex_1_ptr = island.lookup_or_create(uv_vertex_template);
        uv_vertex_template.vertex = shared_edge_vertex;
        uv_vertex_template.uv = old_uv;
        UVVertex *vertex_2_ptr = island.lookup_or_create(uv_vertex_template);
        uv_vertex_template.vertex = other_prim_vertex;
        uv_vertex_template.uv = new_uv;
        UVVertex *vertex_3_ptr = island.lookup_or_create(uv_vertex_template);

        add_uv_primitive_fill(
            island, *vertex_1_ptr, *vertex_2_ptr, *vertex_3_ptr, *fill_primitive);

        segment.flags.found = true;

        UVPrimitive &new_prim = island.uv_primitives[island.uv_primitives.size() - 1];
        current_edge = new_prim.get_uv_edge(uv_vertex->vertex, other_prim_vertex);
        UVBorderEdge new_border(new_prim.get_uv_edge(shared_edge_vertex, other_prim_vertex),
                                &new_prim);
        new_border_edges.append(new_border);
        break;
      }
    }

    {
      /* Add final segment. */
      float2 old_uv = current_edge->get_other_uv_vertex(uv_vertex->vertex)->uv;
      MeshVertex *shared_edge_vertex =
          current_edge->get_other_uv_vertex(uv_vertex->vertex)->vertex;
      MeshPrimitive *fill_primitive = find_fill_border(
          *uv_vertex->vertex, *shared_edge_vertex, *corner.second->get_uv_vertex(1)->vertex);
      BLI_assert(fill_primitive);
      MeshVertex *other_prim_vertex =
          fill_primitive->get_other_uv_vertex(uv_vertex->vertex, shared_edge_vertex)->vertex;

      UVVertex uv_vertex_template;
      uv_vertex_template.vertex = uv_vertex->vertex;
      uv_vertex_template.uv = uv_vertex->uv;
      UVVertex *vertex_1_ptr = island.lookup_or_create(uv_vertex_template);
      uv_vertex_template.vertex = shared_edge_vertex;
      uv_vertex_template.uv = old_uv;
      UVVertex *vertex_2_ptr = island.lookup_or_create(uv_vertex_template);
      uv_vertex_template.vertex = other_prim_vertex;
      uv_vertex_template.uv = corner.second->get_uv_vertex(1)->uv;
      UVVertex *vertex_3_ptr = island.lookup_or_create(uv_vertex_template);
      add_uv_primitive_fill(island, *vertex_1_ptr, *vertex_2_ptr, *vertex_3_ptr, *fill_primitive);

      UVPrimitive &new_prim = island.uv_primitives[island.uv_primitives.size() - 1];
      UVBorderEdge new_border(new_prim.get_uv_edge(shared_edge_vertex, other_prim_vertex),
                              &new_prim);
      new_border_edges.append(new_border);
    }

    int border_insert = corner.first->index;
    border.remove(border_insert);

    int border_next = corner.second->index;
    if (border_next < border_insert) {
      border_insert--;
    }
    else {
      border_next--;
    }
    border.remove(border_next);

    border.edges.insert(border_insert, new_border_edges);

    border.update_indexes(border_index);
  }

  border.update_extendability();
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
  for (UVBorder &border : borders) {
    svg(of, border, step);
  }
  step++;
#endif

  int64_t border_index = 0;
  for (UVBorder &border : borders) {
    border.update_indexes(border_index++);
  }

  // Debug setting to reduce the extension to a number of iterations as long as not all corner
  // cases have been implemented.
  int num_iterations = 99999;
  while (num_iterations) {
    printf("**iterations left %d**\n", num_iterations);
    if (num_iterations == 1) {
      printf("Last iteration\n");
    }
    validate_border();
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
    validate_border();

    /* Mark that the vert is extended. Unable to extend twice. */
    extension_corner->second->flags.extendable = false;
    num_iterations--;
#ifdef DEBUG_SVG
    svg(of, *this, step++);
    for (UVBorder &border : borders) {
      svg(of, border, step);
    }
    step++;
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

bool UVBorder::is_ccw() const
{
  const UVBorderEdge &edge = edges.first();
  const UVVertex *uv_vertex1 = edge.get_uv_vertex(0);
  const UVVertex *uv_vertex2 = edge.get_uv_vertex(1);
  const UVVertex *uv_vertex3 = edge.get_other_uv_vertex();
  float poly[3][2];
  copy_v2_v2(poly[0], uv_vertex1->uv);
  copy_v2_v2(poly[1], uv_vertex2->uv);
  copy_v2_v2(poly[2], uv_vertex3->uv);
  const bool ccw = cross_poly_v2(poly, 3) < 0.0;
  return ccw;
}

void UVBorder::flip()
{
  uint64_t border_index = edges.first().border_index;
  for (UVBorderEdge &edge : edges) {
    edge.reverse_order = !edge.reverse_order;
  }
  std::reverse(edges.begin(), edges.end());
  update_indexes(border_index);
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

void UVBorder::update_extendability()
{
  for (UVBorderEdge &edge : edges) {
    UVBorderEdge &prev_edge = edges[edge.prev_index];
    if (prev_edge.get_uv_vertex(1) != edge.get_uv_vertex(0)) {
      edge.flags.extendable = false;
    }
  }
}

void UVBorder::validate() const
{
  for (const UVBorderEdge &edge : edges) {
    float2 uv1 = edge.get_uv_vertex(0)->uv;
    float2 uv2 = edge.get_uv_vertex(1)->uv;
    std::cout << uv1;
    std::cout << "->";
    std::cout << uv2;
    std::cout << "\n";
  }
  for (const UVBorderEdge &edge : edges) {
    BLI_assert(edges[edge.prev_index].get_uv_vertex(1)->uv == edge.get_uv_vertex(0)->uv);
    BLI_assert(edge.get_uv_vertex(1)->uv == edges[edge.next_index].get_uv_vertex(0)->uv);
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

void UVIslandsMask::dilate(int max_iterations)
{
#ifdef DEBUG_SVG
  std::ofstream of;
  of.open("/tmp/dilate.svg");
  svg_header(of);
#endif

  int index = 0;
  while (index < max_iterations) {
    bool changed = dilate_x(*this);
    changed |= dilate_y(*this);
    if (!changed) {
      break;
    }
#ifdef DEBUG_SVG
    svg(of, *this, index);
#endif
    index++;
  }
#ifdef DEBUG_SVG
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
  return 1024 - uv.y * 1024;
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

  /* Mark Border corners that can be extended. */
  for (const UVBorder &border : island.borders) {
    ss << "    <g fill=\"green\">\n";
    for (const UVBorderEdge &edge : border.edges) {
      if (edge.flags.extendable) {
        const UVVertex *uv_vertex = edge.get_uv_vertex(0);
        ss << "<circle cx=\"" << svg_x(*uv_vertex) << "\"";
        ss << " cy=\"" << svg_y(*uv_vertex) << "\"";
        ss << " r=\"3\" />\n";
      }
    }
    ss << "     </g>\n";
  }

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
      float2 start = float2(float(x), float(y)) / resolution;
      float2 end = float2(float(x), float(y + 1)) / resolution;
      ss << "       <line x1=\"" << svg_x(start) << "\" y1=\"" << svg_y(start) << "\" x2=\""
         << svg_x(end) << "\" y2=\"" << svg_y(end) << "\"/>\n";
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
      float2 start = float2(float(x), float(y)) / resolution;
      float2 end = float2(float(x + 1), float(y)) / resolution;
      ss << "       <line x1=\"" << svg_x(start) << "\" y1=\"" << svg_y(start) << "\" x2=\""
         << svg_x(end) << "\" y2=\"" << svg_y(end) << "\"/>\n";
    }
  }
  ss << " </g>\n";
  ss << "</g>\n";
}

void svg_coords(std::ostream &ss, const float2 &coords)
{
  ss << svg_x(coords) << "," << svg_y(coords);
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

  ss << "<text x=\"" << svg_x(center) << "\"";
  ss << " y=\"" << svg_y(center) << "\">";
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

void svg(std::ostream &ss, const UVBorder &border, int step)
{
  ss << "<g transform=\"translate(" << step * 1024 << " 0)\">\n";

  ss << " <g stroke=\"grey\">\n";
  for (const UVBorderEdge &edge : border.edges) {
    float2 v1 = edge.get_uv_vertex(0)->uv;
    float2 v2 = edge.get_uv_vertex(1)->uv;
    ss << "       <line x1=\"" << svg_x(v1) << "\" y1=\"" << svg_y(v1) << "\" x2=\"" << svg_x(v2)
       << "\" y2=\"" << svg_y(v2) << "\"/>\n";
  }
  ss << " </g>\n";

  ss << " <g fill=\"black\">\n";
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
