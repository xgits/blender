/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2019 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BLI_linklist.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_threads.h"

#include "DNA_lineart_types.h"

#include <math.h>
#include <string.h>

struct LineartEdge;
struct LineartRenderBuffer;
struct LineartStaticMemPool;
struct LineartStaticMemPoolNode;

void *lineart_list_append_pointer_pool(ListBase *list,
                                       struct LineartStaticMemPool *smp,
                                       void *data);
void *lineart_list_append_pointer_pool_sized(ListBase *list,
                                             struct LineartStaticMemPool *smp,
                                             void *data,
                                             int size);
void *lineart_list_append_pointer_pool_thread(ListBase *list,
                                              struct LineartStaticMemPool *smp,
                                              void *data);
void *lineart_list_append_pointer_pool_sized_thread(ListBase *list,
                                                    LineartStaticMemPool *smp,
                                                    void *data,
                                                    int size);
void *list_push_pointer_static(ListBase *list, struct LineartStaticMemPool *smp, void *p);
void *list_push_pointer_static_sized(ListBase *list,
                                     struct LineartStaticMemPool *smp,
                                     void *p,
                                     int size);

void *lineart_list_pop_pointer_no_free(ListBase *list);
void lineart_list_remove_pointer_item_no_free(ListBase *list, LinkData *lip);

struct LineartStaticMemPoolNode *lineart_mem_new_static_pool(struct LineartStaticMemPool *smp,
                                                             size_t size);
void *lineart_mem_acquire(struct LineartStaticMemPool *smp, size_t size);
void *lineart_mem_acquire_thread(struct LineartStaticMemPool *smp, size_t size);
void lineart_mem_destroy(struct LineartStaticMemPool *smp);

void lineart_prepend_pool(LinkNode **first, struct LineartStaticMemPool *smp, void *link);

void lineart_matrix_ortho_44d(double (*mProjection)[4],
                              double xMin,
                              double xMax,
                              double yMin,
                              double yMax,
                              double zMin,
                              double zMax);
void lineart_matrix_perspective_44d(
    double (*mProjection)[4], double fFov_rad, double fAspect, double zMin, double zMax);

int lineart_count_intersection_segment_count(struct LineartRenderBuffer *rb);

void lineart_count_and_print_render_buffer_memory(struct LineartRenderBuffer *rb);

#define LRT_ITER_ALL_LINES_BEGIN \
  { \
    LineartEdge *e; \
    for (int __i = 0; __i < rb->pending_edges.next; __i++) { \
      e = rb->pending_edges.array[__i];

#define LRT_ITER_ALL_LINES_NEXT ; /* Doesn't do anything now with new array setup. */

#define LRT_ITER_ALL_LINES_END \
  LRT_ITER_ALL_LINES_NEXT \
  } \
  }

#define LRT_BOUND_AREA_CROSSES(b1, b2) \
  ((b1)[0] < (b2)[1] && (b1)[1] > (b2)[0] && (b1)[3] < (b2)[2] && (b1)[2] > (b2)[3])

/* Initial bounding area row/column count, setting 10 is tested to be realitvely optimal for the
 * performance under current algorithm. */
#define LRT_BA_ROWS 10

#define LRT_EDGE_BA_MARCHING_BEGIN(fb1, fb2) \
  double x = fb1[0], y = fb1[1]; \
  LineartBoundingArea *ba = lineart_edge_first_bounding_area(rb, fb1, fb2); \
  LineartBoundingArea *nba = ba; \
  double k = (fb2[1] - fb1[1]) / (fb2[0] - fb1[0] + 1e-30); \
  int positive_x = (fb2[0] - fb1[0]) > 0 ? 1 : (fb2[0] == fb1[0] ? 0 : -1); \
  int positive_y = (fb2[1] - fb1[1]) > 0 ? 1 : (fb2[1] == fb1[1] ? 0 : -1); \
  while (nba)

#define LRT_EDGE_BA_MARCHING_NEXT(fb1, fb2) \
  /* Marching along `e->v1` to `e->v2`, searching each possible bounding areas it may touch. */ \
  nba = lineart_bounding_area_next(nba, fb1, fb2, x, y, k, positive_x, positive_y, &x, &y);

#define LRT_EDGE_BA_MARCHING_END

void lineart_main_occlusion_begin(struct LineartRenderBuffer *rb);
void lineart_main_cull_triangles(struct LineartRenderBuffer *rb, bool clip_far);
void lineart_main_free_adjacent_data(struct LineartRenderBuffer *rb);
void lineart_main_perspective_division(struct LineartRenderBuffer *rb);
void lineart_main_discard_out_of_frame_edges(struct LineartRenderBuffer *rb);
void lineart_main_load_geometries(struct Depsgraph *depsgraph,
                                  struct Scene *scene,
                                  struct Object *camera,
                                  struct LineartRenderBuffer *rb,
                                  bool allow_duplicates,
                                  bool do_shadow_casting,
                                  struct ListBase *shadow_elns);
void lineart_main_get_view_vector(struct LineartRenderBuffer *rb);
void lineart_main_bounding_area_make_initial(struct LineartRenderBuffer *rb);
void lineart_main_bounding_areas_connect_post(struct LineartRenderBuffer *rb);
void lineart_main_clear_linked_edges(struct LineartRenderBuffer *rb);
void lineart_main_link_lines(struct LineartRenderBuffer *rb);
void lineart_main_add_triangles(struct LineartRenderBuffer *rb);
bool lineart_main_try_generate_shadow(struct Depsgraph *depsgraph,
                                      struct Scene *scene,
                                      struct LineartRenderBuffer *original_rb,
                                      struct LineartGpencilModifierData *lmd,
                                      struct LineartStaticMemPool *shadow_data_pool,
                                      struct LineartElementLinkNode **r_veln,
                                      struct LineartElementLinkNode **r_eeln,
                                      struct ListBase *r_calculated_edges_eln_list,
                                      struct LineartRenderBuffer **r_shadow_rb_if_reproject);
void lineart_main_make_enclosed_shapes(struct LineartRenderBuffer *rb,
                                       struct LineartRenderBuffer *shadow_rb);
void lineart_main_transform_and_add_shadow(struct LineartRenderBuffer *rb,
                                           struct LineartElementLinkNode *veln,
                                           struct LineartElementLinkNode *eeln);

LineartElementLinkNode *lineart_find_matching_eln(struct ListBase *shadow_elns, int obindex);
LineartEdge *lineart_find_matching_edge(struct LineartElementLinkNode *shadow_eln,
                                        uint64_t edge_identifier);
void lineart_register_shadow_cuts(struct LineartRenderBuffer *rb,
                                  struct LineartEdge *e,
                                  struct LineartEdge *shadow_edge);
void lineart_register_intersection_shadow_cuts(struct LineartRenderBuffer *rb,
                                               struct ListBase *shadow_elns);

bool lineart_edge_from_triangle(const struct LineartTriangle *tri,
                                const struct LineartEdge *e,
                                bool allow_overlapping_edges);
LineartBoundingArea *lineart_edge_first_bounding_area(struct LineartRenderBuffer *rb,
                                                      double *fbcoord1,
                                                      double *fbcoord2);
LineartBoundingArea *lineart_bounding_area_next(struct LineartBoundingArea *_this,
                                                double *fbcoord1,
                                                double *fbcoord2,
                                                double x,
                                                double y,
                                                double k,
                                                int positive_x,
                                                int positive_y,
                                                double *next_x,
                                                double *next_y);
void lineart_edge_cut(struct LineartRenderBuffer *rb,
                      struct LineartEdge *e,
                      double start,
                      double end,
                      uchar material_mask_bits,
                      uchar mat_occlusion,
                      uchar shadow_bits);
void lineart_add_edge_to_array(struct LineartPendingEdges *pe, struct LineartEdge *e);
void lineart_finalize_object_edge_array_reserve(struct LineartPendingEdges *pe, int count);
void lineart_destroy_render_data_keep_init(struct LineartRenderBuffer *rb);

#ifdef __cplusplus
extern "C" {
#endif

void lineart_sort_adjacent_items(struct LineartAdjacentEdge *ai, int length);

#ifdef __cplusplus
}
#endif

#ifndef __cplusplus /* Compatibility code for atomics, only for C. */

#  if defined __has_include /* Try to use C11 atomics support. */
#    if __has_include(<stdatomic.h>)
#      include <stdatomic.h>
#      define lineart_atomic_load(p) atomic_load((volatile size_t *)p)
#      define lineart_atomic_store(p, d) atomic_store((volatile size_t *)p, (size_t)d)
#    endif
#  endif

#  ifdef _MSC_VER /* Atomics walkaround for windows. */
#    define WIN32_LEAN_AND_MEAN
#    include <windows.h>
#    define lineart_atomic_load(p) (MemoryBarrier(), *(p))
#    define lineart_atomic_store(p, d) \
      do { \
        *(p) = (d); \
        MemoryBarrier(); \
      } while (0)
#  endif

#  if !defined lineart_atomic_load /* Fallback */
#    include "atomic_ops.h"
#    define lineart_atomic_load(p) atomic_add_and_fetch_z((size_t *)p, 0)
#    define lineart_atomic_store(p, d) atomic_add_and_fetch_z((size_t *)p, (size_t)d)
#  endif

#endif /* !__cplusplus */
